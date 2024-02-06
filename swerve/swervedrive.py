import abc 
import math
import threading
from collections.abc import Iterable, Sequence, Set
import logging
import wpilib
import wpimath
import wpimath.units
import rev
import navx
from .swervemodule import SwerveModule
from config import *
import time
from typing import NamedTuple, Callable, Any 
import wpimath.kinematics as kinematics
import wpimath.geometry as geom
import wpimath.estimator as estimator
from . import ISwerveDrive, ISwerveModule


class SwerveDrive(ISwerveDrive):
    """Abstract base class for a sweve drive."""
    _modules: dict[ModulePosition, ISwerveModule]

    initialized: bool = False # True if the swerve drive has been initialized at least once

    _kinematics: kinematics.SwerveDrive4Kinematics # Kinematics object for the swerve drive

    logger: logging.Logger
  
    _ordered_modules: list[ISwerveModule]

    _navx: navx.AHRS  # Attitude Heading Reference System

    _odemetry: estimator.SwerveDrive4PoseEstimator
    
    _physical_config: PhysicalConfig

    _odemetry_lock: threading.Lock = threading.Lock()

    @property
    def num_modules(self) -> int:
        return len(self._ordered_modules)
    
    @property
    def modules(self) -> dict[ModulePosition, ISwerveModule]:
        return self._modules 
    
    @property
    def odemetry(self) -> estimator.SwerveDrive4PoseEstimator:
        return self._odemetry
    
    @property
    def ordered_modules(self) -> list[ISwerveModule]:
        """Provides a consistent ordering of modules for use with wpilib swerve functions"""
        return self._ordered_modules
    
    def __init__(self, navx: navx.AHRS, swerve_config: dict[ModulePosition, SwerveModuleConfig], physical_config: PhysicalConfig, logger: logging.Logger):
        self.logger = logger.getChild("swerve")
        self._navx = navx
        self._modules = {}    
        self._physical_config = physical_config
        for position, module_config in swerve_config.items():
            self._modules[position] = SwerveModule(position, module_config, physical_config, self.logger)

        self._ordered_modules = [self.modules[position] for position in sorted(self.modules.keys())]
        self.logger.info(f"Module Order: {[m.id for m in self._ordered_modules]}")
 
        locations = [m.location for m  in self._ordered_modules]
        self._kinematics = kinematics.SwerveDrive4Kinematics(*locations)
 
        module_positions = tuple([m.position for m in self._ordered_modules]) 

        #The Pose Estimator uses the default standard deviations for model and vision estimates.
        # According to wpilib:
        # The default standard deviations of the model states are 
        # 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading. 
        # The default standard deviations of the vision measurements are
        # 0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
        self._odemetry = estimator.SwerveDrive4PoseEstimator(self._kinematics,
                                                             geom.Rotation2d(math.radians(self._navx.getAngle())),
                                                             module_positions, # type: ignore
                                                             geom.Pose2d(0,0,geom.Rotation2d(0)))

        self.initialize()

    
    def initialize(self):
        """Initialize the swerve drive.  Needs to be called repeatedly until it returns True."""
        results = [module.initialize() for module in self._modules.values()]
        if all(results):
            self.initialized = True
            return True

        return False
    
    def periodic(self):
        """Call periodically to update the odemetry"""
        self.update_odometry()
        if __debug__:
            for m in self._ordered_modules:
                m.report_to_dashboard()

    def update_odometry(self):

        with self._odemetry_lock:
            module_positions = tuple([m.position for m in self._ordered_modules])
            self._odemetry.update(geom.Rotation2d.fromDegrees(self._navx.getAngle()),
                                  module_positions) # type: ignore 

    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second,
              run_modules: Sequence[ModulePosition] | Set[ModulePosition] | None = None):
        """Drive the robot using cartesian coordinates

        :param run_modules: A set of modules to drive.  If None, all modules will be driven.  This is useful for testing individual modules and ensuring ModulePosition is correct for each module
        """

        measured_chasis_speeds = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(v_x, v_y, rotation, geom.Rotation2d(
            math.radians(self._navx.getAngle())))
        module_states = self._kinematics.toSwerveModuleStates(measured_chasis_speeds)

        module_states = self._kinematics.desaturateWheelSpeeds(module_states, self._physical_config.max_drive_speed)

        for i in range(self.num_modules):
            module = self._ordered_modules[i]

            if run_modules is not None and module.id not in run_modules:
                continue
 
            state = module_states[i]
            module.desired_state = state

    def stop(self):
        """Set voltage to all motors to zero"""
        for module in self._modules.values():
            module.stop()

    def lock_wheels(self):
        quarter_pi = math.pi / 4.0
        self._modules[ModulePosition.front_left].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(quarter_pi))
        self._modules[ModulePosition.front_right].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(-quarter_pi))
        self._modules[ModulePosition.back_left].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi - quarter_pi))
        self._modules[ModulePosition.back_right].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi + quarter_pi))
  
    @property
    def pose(self) -> geom.Pose2d:
        """Current pose of the robot"""
        with self._odemetry_lock:
            return self._odemetry.getEstimatedPosition()

    @property
    def measured_chassis_speed(self) -> kinematics.ChassisSpeeds:
        """Current chassis speed of the robot"""
        return self._kinematics.toChassisSpeeds(
            tuple([m.measured_state for m in self._ordered_modules]))  # type: ignore

    @property
    def desired_chassis_speed(self) -> kinematics.ChassisSpeeds:
        """chasis speeds desired by the robot"""
        return self._kinematics.toChassisSpeeds(tuple([m.desired_state for m in self._ordered_modules]))

    def add_vision_measurement(self, timestamp: float, pose: geom.Pose2d):
        """Add a vision measurement to the odemetry"""
        with self._odemetry_lock:
            self._odemetry.addVisionMeasurement(pose, timestamp)

    def drive_set_distance(self, meters: float, angle: float):
        for module in self._modules.values():
            module.drive_set_distance(meters, angle)