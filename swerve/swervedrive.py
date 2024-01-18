import math
from collections.abc import Iterable
import logging
import wpilib
import wpimath
import wpimath.units
import rev
from config import *
from . import swervemodule
from .swervemodule import SwerveModule
import time
from typing import NamedTuple, Callable, Any 
import wpimath.kinematics._kinematics as kinematics
import wpimath.geometry as geom


class SwerveDrive():
    '''Abstract base class for a sweve drive.'''
    modules: dict[module_position, SwerveModule]

    initialized: bool = False # True if the swerve drive has been initialized at least once

    kinematics: kinematics.SwerveDrive4Kinematics # Kinematics object for the swerve drive

    logger: logging.Logger

    module_order = list[module_position]
    
    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig, logger: logging.Logger):
        self.logger = logger.getChild("swerve")
        self.modules = {}    
        for position, module_config in swerve_config.items():
            self.modules[position] = SwerveModule(position, module_config, physical_config, self.logger)

        self.module_order = [position for position, module in self.modules.items()]
        locations = [self.modules[position].location for position  in self.module_order]
        self.kinematics = kinematics.SwerveDrive4Kinematics(*locations)
         
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
    
    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second):
        '''Drive the robot using cartesian coordinates'''

        chassis_speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(v_x, v_y, rotation, geom.Rotation2d(0))
        module_states = self.kinematics.toSwerveModuleStates(chassis_speed)

        num_modules = len(self.modules)
        for i in range(num_modules):
            position = self.module_order[i]
            module = self.modules[position]
            state = module_states[i] 
            module.desired_state = state


        pass

    def lockWheels(self):
        quarter_pi = math.pi / 4.0
        self.modules[module_position.front_left].angle = quarter_pi
        self.modules[module_position.front_right].angle = -quarter_pi
        self.modules[module_position.back_left].angle = math.pi - quarter_pi
        self.modules[module_position.back_right].angle = math.pi + quarter_pi

        self.modules[module_position.front_left].velocity = 0
        self.modules[module_position.front_right].velocity = 0
        self.modules[module_position.back_left].velocity = 0
        self.modules[module_position.back_right].velocity = 0
   

class TestConfig(NamedTuple):
    '''A test configuration for a swerve module'''
    duration: float
    test: Callable[..., Any]
    args: Iterable[Any]

class SwerveDriveBasicFunctionTest(SwerveDrive):

    start_time: float
    tests: list[TestConfig]
    current_test: int = 0

    def __init__(self, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig, logger: logging.Logger):
        super().__init__(swerve_config, physical_config, logger)
                         
    def testInit(self):
        self.start_time = time.monotonic() 

        self.tests = [ 
            # TestConfig(1, self.runAngleMotorTest, (module_position.front_left, -0.05)),
            # TestConfig(1, self.runAngleMotorTest, (module_position.front_right, 0.1)),
            # TestConfig(1, self.runAngleMotorTest, (module_position.back_left, -0.15)),
            # TestConfig(1, self.runAngleMotorTest, (module_position.back_right, 0.2)), 
           
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_left, 0)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_right, 0)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_left, 0)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_right, 0)),

            TestConfig(1.25, self.runAngleMotorPIDTests, ([module_position.front_left, 
                                                          module_position.front_right, 
                                                          module_position.back_left,   
                                                          module_position.back_right], 0)),
                                                
            # TestConfig(1, self.runDriveMotorTest, (module_position.front_left, 0.05)),
            # TestConfig(1, self.runDriveMotorTest, (module_position.front_right, 0.05)),
            # TestConfig(1, self.runDriveMotorTest, (module_position.back_left, 0.05)),
            # TestConfig(1, self.runDriveMotorTest, (module_position.back_right, 0.05)),

            #TestConfig(2, self.runDriveMotorRotationTest, (module_position.front_left, 1)),
            #TestConfig(2, self.runDriveMotorRotationTest, (module_position.front_right, 1)),
            #TestConfig(2, self.runDriveMotorRotationTest, (module_position.back_left, 1)),
            # TestConfig(2, self.runDriveMotorRotationTest, (module_position.back_right, 1)),
            
            # TestConfig(3, self.runDriveMotorVelocityTest, (module_position.front_left, 0.5)),
            # TestConfig(3, self.runDriveMotorVelocityTest, (module_position.front_right, -0.5)),
            # TestConfig(3, self.runDriveMotorVelocityTest, (module_position.back_left, 1)),
            # TestConfig(3, self.runDriveMotorVelocityTest, (module_position.back_right, -1)),

            #------ Rotate all motors together ----
            TestConfig(1.25, self.runAngleMotorPIDTests, ([module_position.front_left, 
                                                          module_position.front_right, 
                                                          module_position.back_left,   
                                                          module_position.back_right], math.pi / 2)),
            
            TestConfig(1.25, self.runAngleMotorPIDTests, ([module_position.front_left, 
                                                          module_position.front_right, 
                                                          module_position.back_left,   
                                                          module_position.back_right], math.pi)),
            
            TestConfig(1.25, self.runAngleMotorPIDTests, ([module_position.front_left, 
                                                          module_position.front_right, 
                                                          module_position.back_left,   
                                                          module_position.back_right], -math.pi / 2)),
            #---------------------------------------

            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_left, math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_right, math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_left, math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_right, math.pi / 2)),

            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_left, math.pi)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_right, math.pi)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_left, math.pi)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_right, math.pi)),

            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_left, -math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.front_right, -math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_left, -math.pi / 2)),
            # TestConfig(1.25, self.runAngleMotorPIDTest, (module_position.back_right, -math.pi / 2)),

            TestConfig(1.25, self.lockWheels, ()),

            TestConfig(3, self.runDriveTest, (0.5, 0, 0)),
            TestConfig(3, self.runDriveTest, (0, 0.5, 0)),
            TestConfig(3, self.runDriveTest, (-0.5, 0, 0)),
            TestConfig(3, self.runDriveTest, (0, -0.5, 0)),
           
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

    

    def runDriveMotorTests(self, position: list[module_position], speed):
        for module in position:
            self.runDriveMotorTest(module, speed)
            
    def runDriveMotorTest(self, position: module_position,
                          speed: float):
        self.modules[position].drive_motor.set(speed)

    def runDriveMotorRotationTests(self, num_rotations: list[module_position], speed):
        for module in num_rotations:
            self.runDriveMotorTest(module, speed)
            
    def runDriveMotorRotationTest(self, position: module_position, num_rotations: float):
        self.modules[position].rotate_drive_wheel(num_rotations)

    def runDriveMotorVelocityTest(self, position: module_position, meters_per_sec: float):
        self.modules[position].velocity = meters_per_sec 

    def runAngleMotorTests(self, position: list[module_position],
                          speed: float):
        for module in position:
            self.runAngleMotorTest(module, speed) 
        
    def runAngleMotorTest(self, position: module_position,
                          speed: float):
        self.modules[position].angle_motor.set(speed)

    def runAngleMotorPIDTests(self, position: list[module_position],
                          angle: float):
        for module in position:
            self.runAngleMotorPIDTest(module, angle) 
    
    def runAngleMotorPIDTest(self, position: module_position,
                          angle: float):
        self.modules[position].angle = angle

    def runDriveTest(self, vx, vy, rotation):
        self.drive(vx, vy, rotation)


    