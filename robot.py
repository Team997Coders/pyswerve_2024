import wpilib
import rev 
import config 
import robot_config
import swerve
import debug
import time


class MyRobot(wpilib.TimedRobot):

    swerve_drive: swerve.SwerveDriveBasicFunctionTest

    controller: wpilib.XboxController

    def robotInit(self):
        super().robotInit()
        self.controller = wpilib.XboxController(0)
        self.swerve_drive = swerve.SwerveDriveBasicFunctionTest(robot_config.swerve_modules, robot_config.physical_properties, self.logger)

        self.swerve_drive.initialize()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.swerve_drive.report_to_dashboard()
  
    def teleopPeriodic(self):
        super().teleopPeriodic()

        vx = self.controller.getRawAxis(1)
        vy = self.controller.getRawAxis(0)

        x_deadband = 0.05
        y_deadband = 0.05 

        if(abs(vx) < x_deadband and abs(vy) < y_deadband):
            #TODO: Make sure robot is not actually moving too
            self.swerve_drive.lockWheels()
        else:
            self.swerve_drive.drive(vx, vy, 0)

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
