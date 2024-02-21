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
import config
from config import ModulePosition, PhysicalConfig, SwerveModuleConfig
import time
from typing import NamedTuple, Callable, Any
import wpimath.kinematics as kinematics
import wpimath.geometry as geom
import wpimath.estimator as estimator
from . import ISwerveDrive, ISwerveModule
import commands2
import wpilib.sysid
import math_help


class SwerveDrive(commands2.subsystem.Subsystem):
    """Abstract base class for a sweve drive."""
    _modules: dict[ModulePosition, ISwerveModule]

    initialized: bool = False  # True if the swerve drive has been initialized at least once

    _kinematics: kinematics.SwerveDrive4Kinematics  # Kinematics object for the swerve drive

    logger: logging.Logger

    _ordered_modules: list[ISwerveModule]

    _odemetry: estimator.SwerveDrive4PoseEstimator

    _physical_config: PhysicalConfig

    _odemetry_lock: threading.Lock = threading.Lock()

    #Automatically invert the gyro if the physical properties say it is inverted without
    #having to check the physical properties every time the gyro is accessed.\
    __gyro_get_degrees_lambda: lambda: float

    @property
    def gyro_angle_radians(self) -> wpimath.units.radians:
        return math.radians(self.gyro_angle_degrees)

    @property
    def gyro_angle_degrees(self) -> wpimath.units.degrees:
        return math_help.wrap_angle_degrees(self.__gyro_get_lambda())

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

    def __init__(self, gyro: navx.AHRS, swerve_config: dict[ModulePosition, SwerveModuleConfig],
                 physical_config: PhysicalConfig, logger: logging.Logger):
        super().__init__()
        self.logger = logger.getChild("swerve")
        if physical_config.invert_gyro:
            self.__gyro_get_lambda = lambda: -(gyro.getAngle())
        else:
            self.__gyro_get_lambda = gyro.getAngle
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
                                                             geom.Rotation2d.fromDegrees(self.gyro_angle_degrees),
                                                             module_positions,  # type: ignore
                                                             geom.Pose2d(0,0,geom.Rotation2d(0)))

        self.initialize()

        #Register the subsystem at the end to ensure periodic is called
#        commands2.CommandScheduler.getInstance().registerSubsystem(self)


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
        # if __debug__:
        for m in self._ordered_modules:
            m.report_to_dashboard()  # type: ignore

    def update_odometry(self):
        with self._odemetry_lock:
            self._odemetry.update(geom.Rotation2d.fromDegrees(self.gyro_angle_degrees),  # type: ignore
                                  self._measured_module_positions)  # type: ignore

    def _scale_velocity_to_drive_speed(self, v_x: float, v_y: float) -> tuple[float, float]:
        """Reduce requested speed if it is too fast"""
        requested_speed = math.sqrt(v_x ** 2 + v_y ** 2)

        # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
        if requested_speed > self._physical_config.max_drive_speed:
            scale_factor = self._physical_config.max_drive_speed / requested_speed
            v_x *= scale_factor
            v_y *= scale_factor

        return v_x, v_y

    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second,
              run_modules: Sequence[ModulePosition] | Set[ModulePosition] | None = None):
        """
        Drive the robot using cartesian coordinates
        :param run_modules: A set of modules to drive.  If None, all modules will be driven.  This is useful for testing individual modules and ensuring ModulePosition is correct for each module
        """

        v_x, v_y = self._scale_velocity_to_drive_speed(v_x, v_y)

        # desired_chasis_speeds = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(v_x, v_y, rotation, geom.Rotation2d(
        #    -self.gyro_angle_radians))

        desired_chasis_speeds = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(v_x, v_y, rotation, geom.Rotation2d(
            -self.gyro_angle_radians))  # keep this one

        module_states = self._kinematics.toSwerveModuleStates(desired_chasis_speeds)

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

    @pose.setter
    def pose(self, value: geom.Pose2d):
        with self._odemetry_lock:
            self._odemetry.resetPosition(geom.Rotation2d.fromDegrees(self.gyro_angle_degrees),
                                         # possible cause of teleporting position on field
                                         self._measured_module_positions,
                                         value)


    @property
    def _measured_module_states(self) -> Sequence[kinematics.SwerveModuleState]:
        """Current state of the modules"""
        return tuple([m.measured_state for m in self._ordered_modules])

    @property
    def _measured_module_positions(self) -> Sequence[kinematics.SwerveModuleState]:
        """Current state of the modules"""
        return tuple([m.position for m in self._ordered_modules])

    @property
    def measured_chassis_speed(self) -> kinematics.ChassisSpeeds:
        """Current chassis speed of the robot"""
        return self._kinematics.toChassisSpeeds(self._measured_module_states)  # type: ignore

    @property
    def desired_chassis_speed(self) -> kinematics.ChassisSpeeds:
        """chasis speeds desired by the robot"""
        return self._kinematics.toChassisSpeeds(tuple([m.desired_state for m in self._ordered_modules]))

    def add_vision_measurement(self, pose: geom.Pose2d, timestamp: float):
        """Add a vision measurement to the odemetry"""
        with self._odemetry_lock:
            self._odemetry.addVisionMeasurement(pose, timestamp)

    def drive_set_distance(self, meters: float, angle: float):
        """Drive the robot a certain distance and angle in meters and radians"""
        for module in self._modules.values():
            module.drive_set_distance(meters, angle)

    def drive_at_voltage(self, voltage: float, angle: float):
        """Provide the drive motors a set voltage at the requested angle"""
        for module in self._modules.values():
            module.drive_at_voltage(voltage)

    def log_to_sysid(self, log: wpilib.sysid.SysIdRoutineLog):
        """Log the state of the swerve drive for system identification"""
        for module in self._modules.values():
            module.log_to_sysid(log.motor(str(module.id)))  # type: ignore

    @property
    def velocity(self) -> float:
        """Get the velocity of the robot"""
        chassis_speed = self.measured_chassis_speed
        return math.sqrt(chassis_speed.vx ** 2 + chassis_speed.vy ** 2)