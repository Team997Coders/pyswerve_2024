from __future__ import annotations
import logging
import math
import time
from swerve import SwerveDrive, ModulePosition
from typing import Callable, Iterable, Any, NamedTuple, Sequence

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
    swerve_drive: SwerveDrive

    def __init__(self, swerve_drive: SwerveDrive, logger: logging.Logger):
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
            # TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_left, -0.05)),
            # TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_right, 0.1)),
            # TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_left, -0.15)),
            # TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_right, 0.2)),

            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, 0)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, 0)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, 0)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, 0)),

            # TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
            #                                                ModulePosition.front_right,
            #                                                ModulePosition.back_left,
            #                                                ModulePosition.back_right], 0)),

            # # TestConfig(1, self.runDriveMotorTest, (ModulePosition.front_left, 0.05)),
            # # TestConfig(1, self.runDriveMotorTest, (ModulePosition.front_right, 0.05)),
            # # TestConfig(1, self.runDriveMotorTest, (ModulePosition.back_left, 0.05)),
            # # TestConfig(1, self.runDriveMotorTest, (ModulePosition.back_right, 0.05)),

            # TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_left, 1)),
            # TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_right, 1)),
            # TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_left, 1)),
            # TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_right, 1)),

            # # TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.front_left, 0.5)),
            # # TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.front_right, -0.5)),
            # # TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.back_left, 1)),
            # # TestConfig(3, self.runDriveMotorVelocityTest, (ModulePosition.back_right, -1)),

            # # # ------ Rotate all motors together ----
            # TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
            #                                                ModulePosition.front_right,
            #                                                ModulePosition.back_left,
            #                                                ModulePosition.back_right], math.pi / 2)),

            # TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
            #                                                ModulePosition.front_right,
            #                                                ModulePosition.back_left,
            #                                                ModulePosition.back_right], math.pi)),

            # TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
            #                                                ModulePosition.front_right,
            #                                                ModulePosition.back_left,
            #                                                ModulePosition.back_right], -math.pi / 2)),
            # # # ---------------------------------------

            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi / 2)),

            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi)),

            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, -math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, -math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, -math.pi / 2)),
            # # TestConfig(1.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, -math.pi / 2)),

            # TestConfig(1.25, self.swerve_drive.lock_wheels, ()),

            # Drive individual modules in translation in various directions
            # TestConfig(3, self.runDriveTest, (0.5, 0, 0.2, [ModulePosition.front_left])),
            # TestConfig(3, self.runDriveTest, (0.5, 0, 0.2, [ModulePosition.front_right])),
            # TestConfig(3, self.runDriveTest, (0.5, 0, 0.2, [ModulePosition.back_right])),
            # TestConfig(3, self.runDriveTest, (0.5, 0, 0.2, [ModulePosition.back_left])),

            #Drive using translation in various directions
            TestConfig(5, self.runDriveTest, (1, 0, 0)),
            # TestConfig(3, self.runDriveTest, (0, 0.5, 0)),
            TestConfig(5, self.runDriveTest, (-1, 0, 0)),
            # TestConfig(3, self.runDriveTest, (0, -0.5, 0)),

            #Drive using rotation in both directions
            # TestConfig(3, self.runDriveTest, (0, 0, 0.5)),
            # TestConfig(3, self.runDriveTest, (0, 0, -0.5)),

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
        '''Turn on the drive motors to a specific power.  Ensures drive motors are functional.'''
        for module in position:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorTest(self, position: ModulePosition,
                          speed: float):
        '''Turn on the drive motor to a specific power.  Ensures drive motors are functional.'''
        self.logger.info(f"{position} drive motor should be moving {speed * 100} % power")
        self.swerve_drive.modules[position].drive_motor.set(speed)

    def runDriveMotorRotationTests(self, num_rotations: list[ModulePosition], speed: float):
        '''Rotates the drive wheels a specific number of rotations.  Ensures our wheel size, gear ratio and math
           are correct.  This is important for odometry to measure progress across the field.'''
        for module in num_rotations:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorRotationTest(self, position: ModulePosition, num_rotations: float):
        '''Rotates the drive wheel a specific number of rotations.  Ensures our wheel size, gear ratio and math
           are correct.  This is important for odometry to measure progress across the field.'''
        self.logger.info(f"{position} drive motor wheel should rotate {num_rotations} time(s)")
        self.swerve_drive.modules[position].rotate_drive_wheel(num_rotations)

    def runDriveMotorVelocityTest(self, position: ModulePosition, meters_per_sec: float): 
        '''Turn on the drive motor to a specific speed.  Ensures that drive motors move at the 
        expected speed.  This can be hard to precisely measure, but eyeballing or using a stopwatch as it drives across ground is a start.'''
        self.logger.info(f"{position} drive motor should be moving {meters_per_sec} m/s")
        self.swerve_drive.modules[position].velocity = meters_per_sec

    def runAngleMotorTests(self, position: list[ModulePosition],
                           power: float):
        '''Turn on the angle motors to a specific power.  Ensures angle motors are functional.'''
        for module in position:
            self.runAngleMotorTest(module, power)

    def runAngleMotorTest(self, position: ModulePosition,
                          power: float):
        '''Turn on the angle motor to a specific power.  Ensures angle motors are functional.'''
        self.logger.info(f"{position} angle motor should be spinning at {power * 100} % power")
        self.swerve_drive.modules[position].angle_motor.set(power)

    def runAngleMotorPIDTests(self, position: list[ModulePosition],
                              angle: float):
        '''Set the angles of the motors, does not pass through optimization.  Ensures feedback sensors and pid are correctly set.'''
        for module in position:
            self.runAngleMotorPIDTest(module, angle)

    def runAngleMotorPIDTest(self, position: ModulePosition,
                             angle: float):
        '''Set the angle of the motor, does not pass through optimization.   Ensures feedback sensors and pid are correctly set.'''
        self.logger.info(f"{position} angle motor should move to {math.degrees(angle)} degrees")
        self.swerve_drive.modules[position].angle = angle

    def runDriveTest(self, vx, vy, rotation, run_modules: Sequence[ModulePosition] | None = None):
        '''
        Set a desired drive state.  Output should be optimized to use the minimal angle

        :param run_modules: A set of modules to drive.  If None, all modules will be driven.  This is useful for testing individual modules and ensuring ModulePosition is correct for each module
        '''
        if run_modules is None:
            self.logger.info(f"Chassis should be driving at {vx}x  {vy}y {rotation} rotation rate")
        else:
            self.logger.info(f"The modules below should be driving at {vx}x  {vy}y {rotation} rotation rate:")
            for m in run_modules:
                self.logger.info(f"\t{m}")
            
        self.swerve_drive.drive(vx, vy, rotation, run_modules)

        #TODO: Test the drive states to ensure the angle and direction of each wheel is correct

