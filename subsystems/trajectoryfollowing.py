import math

import rev
import wpilib
import wpimath.geometry as geom
from commands2 import Subsystem

import config
import swerve
from swerve import SwerveDrive
import commands2
import pathplannerlib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from config import ModulePosition

class TrajectoryFollowing(Subsystem):
    _swerve_drive: swerve.SwerveDrive

    def __init__(self, swerve_drive: swerve.SwerveDrive,
                       axis_config: config.ProfiledPIDConfig,
                       heading_config: config.PIDConfig) -> None:
        self._swerve_drive = swerve_drive
        #This code assumes all modules are equidistant from the center
        front_left_module = swerve_drive.modules[ModulePosition.front_left]
        wheel_location = front_left_module.location
        drive_base_radius = math.sqrt(wheel_location.x ** 2 + wheel_location.y ** 2)
        super().__init__()
        AutoBuilder.configureHolonomic(
            lambda: self._swerve_drive.pose,  # Robot pose supplier
            self._swerve_drive.reset_pose,  # Method to reset odometry (will be called if your auto has a starting pose)
            lambda: self._swerve_drive.measured_chassis_speed,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self._swerve_drive.drive_with_chassis_speeds,
            # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(axis_config.p,
                             axis_config.i,
                             axis_config.d),  # Translation PID constants
                PIDConstants(heading_config.p,
                             heading_config.i,
                             heading_config.d),  # Rotation PID constants
                axis_config.profile.velocity,  # Max module speed, in m/s
                drive_base_radius,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig()  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
