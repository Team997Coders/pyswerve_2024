import sys
import wpilib
import rev
import config
from config import AxisConfig
import robot_config
import swerve
import time
import ntcore
import telemetry
import math
import navx
from drivers import TestDriver
from drivers import TeleopDrive
from swerve import SwerveDrive
from debug import attach_debugger
import wpimath.kinematics as kinematics
from wpilib import SmartDashboard, Field2d
from wpilib import SmartDashboard as sd
import commands2
from math_helper import Range
from math_help import processControllerDeadband

if __debug__ and "run" in sys.argv:
    # To enter debug mode, add the --debug flag to the 'deploy' command:
    # python -m robotpy deploy --debug
    # At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger()


class MyRobot(commands2.TimedCommandRobot):
    # _command_scheduler: commands2.CommandScheduler

    swerve_drive: SwerveDrive
    swerve_telemetry: telemetry.SwerveTelemetry
    test_driver: TestDriver
    teleop_drive: TeleopDrive
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: wpilib.XboxController
    joystick_one: wpilib.Joystick
    joystick_two: wpilib.Joystick

    photonvision: ntcore.NetworkTable | None

    field: wpilib.Field2d

    @property
    def navx(self) -> navx.AHRS:
        return self._navx

    def robotInit(self):
        super().robotInit()
        # self._command_scheduler = commands2.CommandScheduler()
        self._navx = navx.AHRS.create_spi()
        self.controller = wpilib.XboxController(0)
        self.joystick_one = wpilib.Joystick(0)
        self.joystick_two = wpilib.Joystick(1)
        self.field = wpilib.Field2d()
        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules,
                                               robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)
        SmartDashboard.putData("Field", self.field)
        self.swerve_drive.initialize()

        try:
            self.photonvision = ntcore.NetworkTableInstance.getDefault().getTable("photonvision/Camera_Module_v2")
            if self.photonvision is not None:
                self.logger.info(f"Photonvision connected!")
            else:
                self.logger.error(f"Could not connect to PhotonVision.")
        except Exception as e:
            self.logger.error(f"Could not connect to PhotonVision.\n{e}")
            self.photonvision = None

        self.test_driver = TestDriver(self.swerve_drive, self.logger)
        self.teleop_drive = TeleopDrive(self.swerve_drive,
                                        AxisConfig(input_range=robot_config.joystick_controls.x_deadband,
                                                   output_range=Range(0, robot_config.physical_properties.max_drive_speed),
                                                   controller=self.joystick_one,
                                                   axis_index=0),
                                        AxisConfig(input_range=robot_config.joystick_controls.y_deadband,
                                                   output_range=Range(0, robot_config.physical_properties.max_drive_speed),
                                                   controller=self.joystick_one,
                                                   axis_index=1),
                                        AxisConfig(input_range=robot_config.joystick_controls.theta_deadband,
                                                   output_range=Range(0, robot_config.physical_properties.max_rotation_speed),
                                                   controller=self.joystick_two,
                                                   axis_index=0))

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.swerve_telemetry.report_to_dashboard()
        self.swerve_drive.periodic()
        self.update_position()
        self.field.setRobotPose(self.swerve_drive.pose)
        sd.putData("Field", self.field)

    def update_position(self) -> bool:

        if self.photonvision is None:
            return False

        has_qr_code = self.photonvision.getEntry("hasTarget").getBoolean(False)

        if has_qr_code:
            self.logger.info(f"PhotonVision has_qr_code: {has_qr_code}")
            # TODO: Update pose position using angle to april tag and distance
            # self.swerve_drive.odemetry.resetPosition(kinematics.SwerveModulePosition(0,0,0), geom.Rotation2d(0))
            return True

        return False

    def teleopPeriodic(self):
        super().teleopPeriodic()
        self.teleop_drive.drive()

    def updateField(self):
        pass

    def autonomousInit(self):
        super().autonomousInit()

    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()
        self.test_driver.testInit()

    def testPeriodic(self) -> None:
        super().testPeriodic()
        self.test_driver.testPeriodic()