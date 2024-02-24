import rev
import wpilib
import wpimath.geometry as geom
from commands2 import Subsystem

import swerve
from swerve import SwerveDrive
import commands2
import pathplannerlib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants




class TrajectoryFollwing(Subsystem):
    _swerve_drive: swerve.SwerveDrive
    def __init__(self, swerve_drive: swerve.SwerveDrive) -> None:

        self._swerve_drive = swerve_drive
        AutoBuilder.configureHolonomic(
            lambda: self._swerve_drive.pose, # Robot pose supplier
            self._swerve_drive.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            lambda: self._swerve_drive.measured_chassis_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self._swerve_drive.drive_with_chassis_speeds, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(.18, 0.120, 0.001), # Translation PID constants
                PIDConstants(0.18, 0.12, 0.001), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed



    


    