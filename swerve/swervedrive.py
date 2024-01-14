import math
import wpilib
import rev
from config import *
from . import swervemodule
from .swervemodule import SwerveModule
import time
from typing import NamedTuple, Callable, Any

class SwerveDrive():
    '''Abstract base class for a sweve drive.'''
    modules: dict[module_position, SwerveModule]

    initialized: bool = False # True if the swerve drive has been initialized at least once
    
    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig):
        self.modules = {}    
        for position, module_config in swerve_config.items():
            self.modules[position] = SwerveModule(position, module_config, physical_config)

        self.initialize()

    def report_to_dashboard(self):
        '''Write all module info to nettables'''
        for module in self.modules.values():
            module.report_to_dashboard()

    def initialize(self):
        '''Initialize the swerve drive.  Needs to be called repeatedly until it returns True.'''
        results = [module.init_rotation() for module in self.modules.values()]
        if all(results):
            self.initialized = True
            return True

        return False
           

class TestConfig(NamedTuple):
    '''A test configuration for a swerve module'''
    duration: float
    test: Callable[[module_position, Any], None]
    args: tuple[module_position, Any]

class SwerveDriveBasicFunctionTest(SwerveDrive):

    start_time: float
    tests: list[TestConfig]
    current_test: int = 0

    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig):
        super().__init__(swerve_config, physical_config)
                         
    def testInit(self):
        self.start_time = time.monotonic() 

        self.tests = [
            

            #TestConfig(1, self.runAngleMotorTest, (module_position.front_left, 0.05)),
            #TestConfig(1, self.runAngleMotorTest, (module_position.front_right, 0.05)),
            #TestConfig(1, self.runAngleMotorTest, (module_position.back_left, 0.05)),
            #TestConfig(1, self.runAngleMotorTest, (module_position.back_right, 0.05)), 
           
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_left, 0)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_right, 0)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_left, 0)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_right, 0)),

            TestConfig(1, self.runDriveMotorTest, (module_position.front_left, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (module_position.front_right, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (module_position.back_left, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (module_position.back_right, 0.05)),

            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_left, math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_right, math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_left, math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_right, math.pi / 2)),

            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_left, math.pi)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_right, math.pi)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_left, math.pi)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_right, math.pi)),

            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_left, -math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.front_right, -math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_left, -math.pi / 2)),
            TestConfig(2, self.runAngleMotorPIDTest, (module_position.back_right, -math.pi / 2))
        ]

        test_config = self.tests[self.current_test]
        test_config.test(*test_config.args)

    def testPeriodic(self): 
        elapsed_time = time.monotonic() - self.start_time
        if elapsed_time > self.tests[self.current_test].duration:
            self.current_test += 1
            self.start_time = time.monotonic()
            if self.current_test >= len(self.tests):
                self.current_test = 0

            self.stopMotors()
            
            test_config = self.tests[self.current_test]
            test_config.test(*test_config.args)

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
    