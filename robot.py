import wpilib
import rev 
import config 
import robot_config
import swerve
import debug
import time
import telemetry
import math
import navx


class MyRobot(wpilib.TimedRobot):

    swerve_drive: swerve.SwerveDriveBasicFunctionTest
    swerve_telemetry: telemetry.SwerveTelemetry
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: wpilib.XboxController

    @property
    def navx(self) -> navx.AHRS:
        return self._navx

    def robotInit(self):
        super().robotInit()
        self._navx = navx.AHRS.create_spi()
        self.controller = wpilib.XboxController(0)
        self.swerve_drive = swerve.SwerveDriveBasicFunctionTest(self._navx, robot_config.swerve_modules, robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)

        self.swerve_drive.initialize()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.swerve_telemetry.report_to_dashboard()
  
    def teleopPeriodic(self):
        super().teleopPeriodic()

        vx = self.controller.getRawAxis(1)
        vy = self.controller.getRawAxis(0)

        x_deadband = 0.05
        y_deadband = 0.05 

        if abs(vx) < x_deadband and abs(vy) < y_deadband:
            # TODO: Make sure robot is not actually moving too
            self.swerve_drive.lockWheels()
        else:
            vx *= robot_config.physical_properties.max_drive_speed
            vy *= robot_config.physical_properties.max_drive_speed
            scaled_speed = math.sqrt(vx * vx + vy * vy) 

            # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
            if scaled_speed > robot_config.physical_properties.max_drive_speed:
                scale_factor = robot_config.physical_properties.max_drive_speed / scaled_speed
                vx *= scale_factor
                vy *= scale_factor

            self.swerve_drive.drive(-vx, -vy, 0)

    def autonomousInit(self):
        super().autonomousInit()

    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()  
        self.swerve_drive.testInit()
 
    def testPeriodic(self) -> None:
        super().testPeriodic()
        self.swerve_drive.testPeriodic()
