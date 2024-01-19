import abc 
import math
from collections.abc import Iterable
import logging
import wpilib
import wpimath
import wpimath.units
import rev
import navx
from config import *
from . import swervemodule
from .swervemodule import SwerveModule, ISwerveModule
import time
from typing import NamedTuple, Callable, Any 
import wpimath.kinematics as kinematics
import wpimath.geometry as geom

class ISwerveDrive(abc.ABC):
    @abc.abstractmethod
    def num_modules(self) -> int:
        '''Returns the number of swerve modules'''
        raise NotImplementedError()

    '''Interface for a swerve drive'''
    @abc.abstractmethod
    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second):
        '''Drive the robot using cartesian coordinates'''
        raise NotImplementedError()

    @abc.abstractmethod
    def lockWheels(self):
        '''Lock the wheels in place'''
        raise NotImplementedError()
    
    @abc.abstractproperty
    def modules(self) -> dict[module_position, ISwerveModule]:
        '''Returns a dictionary of swerve modules'''
        raise NotImplementedError()
    
    @abc.abstractproperty
    def ordered_modules(self) -> list[ISwerveModule]:
        '''Returns a list of swerve modules in order.  This order should not change after initialization as it is passed to wpilib functions that depend on consistent ordering'''
        raise NotImplementedError()
    
    @abc.abstractmethod
    def initialize(self) -> bool:
        '''Initialize the swerve drive.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True.'''
        raise NotImplementedError()


class SwerveDrive(ISwerveDrive):
    '''Abstract base class for a sweve drive.'''
    _modules: dict[module_position, ISwerveModule]

    initialized: bool = False # True if the swerve drive has been initialized at least once

    _kinematics: kinematics.SwerveDrive4Kinematics # Kinematics object for the swerve drive

    logger: logging.Logger
  
    _ordered_modules: list[ISwerveModule]

    _navx: navx.AHRS  # Attitude Heading Reference System

    @property
    def num_modules(self) -> int:
        return len(self._ordered_modules)
    
    @property
    def modules(self) -> dict[module_position, ISwerveModule]:
        return self._modules 
    
    @property
    def ordered_modules(self) -> list[ISwerveModule]:
        '''Provides a consistent ordering of modules for use with wpilib swerve functions'''
        return self._ordered_modules
    
    def __init__(self, navx: navx.AHRS, swerve_config: dict[module_position, SwerveModuleConfig], physical_config: PhysicalConfig, logger: logging.Logger):
        self.logger = logger.getChild("swerve")
        self._navx = navx
        self._modules = {}    
        for position, module_config in swerve_config.items():
            self._modules[position] = SwerveModule(position, module_config, physical_config, self.logger)

        self._ordered_modules = [self.modules[position] for position in sorted(self.modules.keys())]
 
        locations = [m.location for m  in self._ordered_modules]
        self._kinematics = kinematics.SwerveDrive4Kinematics(*locations)
         
        self.initialize() 

    
    def initialize(self):
        '''Initialize the swerve drive.  Needs to be called repeatedly until it returns True.'''
        results = [module.initialize() for module in self._modules.values()]
        if all(results):
            self.initialized = True
            return True

        return False
    
    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second):
        '''Drive the robot using cartesian coordinates'''

        chassis_speed = kinematics.ChassisSpeeds.fromRobotRelativeSpeeds(v_x, v_y, rotation, geom.Rotation2d(math.radians(self._navx.getAngle())))
        module_states = self._kinematics.toSwerveModuleStates(chassis_speed)

        num_modules = len(self._modules)
        for i in range(num_modules):
            module = self.ordered_modules[i]
            position = module.position 
            state = module_states[i] 
            module.desired_state = state  


    def lockWheels(self):
        quarter_pi = math.pi / 4.0
        self._modules[module_position.front_left].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(quarter_pi))
        self._modules[module_position.front_right].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(-quarter_pi))
        self._modules[module_position.back_left].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi - quarter_pi))
        self._modules[module_position.back_right].desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(math.pi + quarter_pi))

        # self._modules[module_position.front_left].angle = quarter_pi
        # self._modules[module_position.front_right].angle = -quarter_pi
        # self._modules[module_position.back_left].angle = math.pi - quarter_pi
        # self._modules[module_position.back_right].angle = math.pi + quarter_pi

        # self._modules[module_position.front_left].velocity = 0
        # self._modules[module_position.front_right].velocity = 0
        # self._modules[module_position.back_left].velocity = 0
        # self._modules[module_position.back_right].velocity = 0
  

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
            TestConfig(1, self.runAngleMotorTest, (module_position.front_left, -0.05)),
            TestConfig(1, self.runAngleMotorTest, (module_position.front_right, 0.1)),
            TestConfig(1, self.runAngleMotorTest, (module_position.back_left, -0.15)),
            TestConfig(1, self.runAngleMotorTest, (module_position.back_right, 0.2)), 
           
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
        for module in self._modules.values():
            module.angle_motor.set(0)
            module.drive_motor.set(0)

    

    def runDriveMotorTests(self, position: list[module_position], speed):
        for module in position:
            self.runDriveMotorTest(module, speed)
            
    def runDriveMotorTest(self, position: module_position,
                          speed: float):
        self._modules[position].drive_motor.set(speed)

    def runDriveMotorRotationTests(self, num_rotations: list[module_position], speed):
        for module in num_rotations:
            self.runDriveMotorTest(module, speed)
            
    def runDriveMotorRotationTest(self, position: module_position, num_rotations: float):
        self._modules[position].rotate_drive_wheel(num_rotations)

    def runDriveMotorVelocityTest(self, position: module_position, meters_per_sec: float):
        self._modules[position].velocity = meters_per_sec 

    def runAngleMotorTests(self, position: list[module_position],
                          speed: float):
        for module in position:
            self.runAngleMotorTest(module, speed) 
        
    def runAngleMotorTest(self, position: module_position,
                          speed: float):
        self._modules[position].angle_motor.set(speed)

    def runAngleMotorPIDTests(self, position: list[module_position],
                          angle: float):
        for module in position:
            self.runAngleMotorPIDTest(module, angle) 
    
    def runAngleMotorPIDTest(self, position: module_position,
                          angle: float):
        self._modules[position].angle = angle

    def runDriveTest(self, vx, vy, rotation):
        self.drive(vx, vy, rotation)


    