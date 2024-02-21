import rev
import wpilib
import wpimath.geometry as geom
import swerve
from swerve import SwerveDrive
import commands2
import pathplannerlib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants


class TrajectoryFollwing(commands2.Command):
    _swerve_drive: swerve.SwerveDrive
    def __init__(self, swerve_drive: swerve.SwerveDrive) -> None:

        self._swerve_drive = swerve_drive
        AutoBuilder.configureHolonomic(
            lambda: self._swerve_drive.pose, # Robot pose supplier
            self._swerve_drive.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            lambda: self._swerve_drive.measured_chassis_speed, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self._swerve_drive.drive, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(5.0, 0.0, 0.0), # Translation PID constants
                PIDConstants(5.0, 0.0, 0.0), # Rotation PID constants
                4.5, # Max module speed, in m/s
                0.4, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )





    


    