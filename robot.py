import wpilib
import rev 
import config 
import robot_config
import swerve
import debug
import time


class MyRobot(wpilib.TimedRobot):

    swerve_drive: swerve.SwerveDriveBasicFunctionTest

    def robotInit(self):
        super().robotInit()
        self.swerve_drive = swerve.SwerveDriveBasicFunctionTest(robot_config.swerve_modules, robot_config.physical_properties, self.logger)

        self.swerve_drive.initialize()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.swerve_drive.report_to_dashboard()
  
    def teleopPeriodic(self):
        super().teleopPeriodic()

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
