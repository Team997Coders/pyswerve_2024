import abc
import wpilib
import rev
from config import *
from . import swervemodule
from .swervemodule import SwerveModule
import time

class SwerveDrive(abc.ABC):
    '''Abstract base class for a sweve drive.'''
    modules: dict[module_position, SwerveModule]
    
    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig):
        self.modules = {}    
        for position, module_config in swerve_config.items():
            self.modules[position] = SwerveModule(position, module_config, physical_config)

    def report_to_dashboard(self):
        '''Write all module info to nettables'''
        for module in self.modules.values():
            module.report_to_dashboard()

class SwerveDriveBasicFunctionTest(SwerveDrive):

    start_time: float
    tests: list
    current_test: int = 0

    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig):
        super().__init__(swerve_config, physical_config)
                         
    def testInit(self):
        self.start_time = time.monotonic() 

        self.tests = [
            (self.runDriveMotorTest, (module_position.front_left, 0.05)),
            (self.runDriveMotorTest, (module_position.front_right, 0.05)),
            (self.runDriveMotorTest, (module_position.back_left, 0.05)),
            (self.runDriveMotorTest, (module_position.back_right, 0.05)),
            (self.runAngleMotorTest, (module_position.front_left, 0.05)),
            (self.runAngleMotorTest, (module_position.front_right, 0.05)),
            (self.runAngleMotorTest, (module_position.back_left, 0.05)),
            (self.runAngleMotorTest, (module_position.back_right, 0.05)),
            (self.runAngleMotorPIDTest, (module_position.front_left, 0)),
            (self.runAngleMotorPIDTest, (module_position.front_right, 0)),
            (self.runAngleMotorPIDTest, (module_position.back_left, 0)),
            (self.runAngleMotorPIDTest, (module_position.back_right, 0)),
        ]

        test_data = self.tests[self.current_test]
        test_data[0](*test_data[1])

    def testPeriodic(self): 
        elapsed_time = time.monotonic() - self.start_time
        if elapsed_time > 1:
            self.current_test += 1
            self.start_time = time.monotonic()
            if self.current_test >= len(self.tests):
                self.current_test = 0

            self.stopMotors()
            
            test_data = self.tests[self.current_test]
            test_data[0](*test_data[1])
        
    

    def stopMotors(self):
        '''Turn off the motors to prepare for the next test'''
        for module in self.modules.values():
            module.angle_motor.set(0)
            module.drive_motor.set(0)
            
    def runDriveMotorTest(self, position: module_position,
                          speed: float):
        self.modules[position].drive_motor.set(speed)
        

    def runAngleMotorTest(self, position: module_position,
                          speed: float):
        self.modules[position].angle_motor.set(speed)
    
    def runAngleMotorPIDTest(self, position: module_position,
                          angle: float):
        self.modules[position].set_angle(angle)
    