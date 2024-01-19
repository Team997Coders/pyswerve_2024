from __future__ import annotations
import logging
import math
import time
from swerve import ISwerveDrive, ModulePosition
from typing import Callable, Iterable, Any, NamedTuple

class TestConfig(NamedTuple):
    '''A test configuration for a swerve module'''
    duration: float
    test: Callable[..., Any]
    args: Iterable[Any]

class TestGroup(NamedTuple):
    tests: list[TestConfig | TestGroup]
    name: str # Name of the test group
    description: str # Description of expected outcome of running the test

#Tests refer to functions on the SwerveTestDriver class.  They can be organized into lists and then different lists of tests exercise different functions of the robot drivetrain.


class TestDriver:
    '''
    This class automatically drives the robot through a series of tests.  It is intended to be used to ensure hardware is properly configured.
    If these tests behave as expected the swerve drive should operate correctly.
    '''
    start_time: float
    tests: list[TestConfig]
    current_test: int = 0
    logger: logging.Logger
    swerve_drive: ISwerveDrive

    def __init__(self, swerve_drive: ISwerveDrive, logger: logging.Logger):
        super().__init__()
        self.swerve_drive = swerve_drive
        self.logger = logger.getChild("SwerveTestDriver")


    def testInit(self):
        self.start_time = time.monotonic()

        #Expected Outcome: Each wheel should rotate in the same direction
        self._rotation_tests = [
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_left, 0.1)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_right, 0.1)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_left, 0.1)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_right, 0.1)),
        ]

        #Expected Outcome: Each wheel should rotate to align with the robots forward direction
        self._angle_motor_offset_tests = [
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, 0)),
        ]

        self.tests = [
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_left, -0.05)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_right, 0.1)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_left, -0.15)),
            TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_right, 0.2)),

            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, 0)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, 0)),

            TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                           ModulePosition.front_right,
                                                           ModulePosition.back_left,
                                                           ModulePosition.back_right], 0)),

            TestConfig(1, self.runDriveMotorTest, (ModulePosition.front_left, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (ModulePosition.front_right, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (ModulePosition.back_left, 0.05)),
            TestConfig(1, self.runDriveMotorTest, (ModulePosition.back_right, 0.05)),

            TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_left, 1)),
            TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_right, 1)),
            TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_left, 1)),
            TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_right, 1)),

            TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.front_left, 0.5)),
            TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.front_right, -0.5)),
            TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.back_left, 1)),
            TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.back_right, -1)),

            # ------ Rotate all motors together ----
            TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                           ModulePosition.front_right,
                                                           ModulePosition.back_left,
                                                           ModulePosition.back_right], math.pi / 2)),

            TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                           ModulePosition.front_right,
                                                           ModulePosition.back_left,
                                                           ModulePosition.back_right], math.pi)),

            TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                           ModulePosition.front_right,
                                                           ModulePosition.back_left,
                                                           ModulePosition.back_right], -math.pi / 2)),
            # ---------------------------------------

            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi / 2)),

            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi)),

            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, -math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, -math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, -math.pi / 2)),
            TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, -math.pi / 2)),

            TestConfig(1.25, self.swerve_drive.lock_wheels, ()),

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
        self.swerve_drive.stop() 

    def runDriveMotorTests(self, position: list[ModulePosition], speed):
        for module in position:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorTest(self, position: ModulePosition,
                          speed: float):
        self.logger.info(f"{position} drive motor should be moving {speed * 100} % power")
        self.swerve_drive.modules[position].drive_motor.set(speed)

    def runDriveMotorRotationTests(self, num_rotations: list[ModulePosition], speed):
        for module in num_rotations:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorRotationTest(self, position: ModulePosition, num_rotations: float):
        
        self.logger.info(f"{position} drive motor wheel should rotate {num_rotations} time(s)")
        self.swerve_drive.modules[position].rotate_drive_wheel(num_rotations)

    def runDriveMotorVelocityTest(self, position: ModulePosition, meters_per_sec: float): 
        self.logger.info(f"{position} drive motor should be moving {meters_per_sec} m/s")
        self.swerve_drive.modules[position].velocity = meters_per_sec

    def runAngleMotorTests(self, position: list[ModulePosition],
                           speed: float):
        for module in position:
            self.runAngleMotorTest(module, speed)

    def runAngleMotorTest(self, position: ModulePosition,
                          speed: float):
        self.logger.info(f"{position} angle motor should be spinning at {speed * 100} % power")
        self.swerve_drive.modules[position].angle_motor.set(speed)

    def runAngleMotorPIDTests(self, position: list[ModulePosition],
                              angle: float):
        for module in position:
            self.runAngleMotorPIDTest(module, angle)

    def runAngleMotorPIDTest(self, position: ModulePosition,
                             angle: float):
        self.logger.info(f"{position} angle motor should move to {math.degrees(angle)} degrees")
        self.swerve_drive.modules[position].angle = angle

    def runDriveTest(self, vx, vy, rotation):
        self.logger.info(f"Chassis should be driving at {vx}x  {vy}y {rotation} rotation rate")
        self.swerve_drive.drive(vx, vy, rotation)
