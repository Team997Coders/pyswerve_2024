from __future__ import annotations
import logging
import math
import time

import wpilib

import robot
from swerve import SwerveDrive, ModulePosition
from typing import Callable, Iterable, Any, NamedTuple, Sequence


class TestConfig(NamedTuple):
    """A test configuration for a swerve module"""
    duration: float
    test: Callable[..., Any]
    args: Iterable[Any]


class TestGroup(NamedTuple):
    name: str  # Name of the test group
    description: str  # Description of expected outcome of running the test
    tests: list[TestConfig]  # Todo: enable having TestGroups in the list as well


def create_test_selection_widget(sd_path: str, test_groups: list[TestGroup]) -> wpilib.SendableChooser:
    """Creates a widget in smart dashboard that can select which test group to run from a list
    :param sd_path: The path in smart dashboard to write the selected test group to
    """
    chooser = wpilib.SendableChooser()
    for i, tg in enumerate(test_groups):
        chooser.addOption(tg.name, i)  # Write the index to the dashboard, we'll use this to lookup which test to run

    tg = test_groups[-1]  # The last test in the test group is the default
    chooser.setDefaultOption(tg.name, len(test_groups) - 1)
    wpilib.SmartDashboard.putData(sd_path, chooser)
    return chooser


# Tests refer to functions on the SwerveTestDriver class.  They can be organized into lists and then different lists of tests exercise different functions of the robot drivetrain.


class TestDriver:
    """
    This class automatically drives the robot through a series of tests.  It is intended to be used to ensure hardware is properly configured.
    If these tests behave as expected the swerve drive should operate correctly.
    """
    start_time: float | None
    _current_test_group_index: int = 0
    _current_test: int = 0  #
    logger: logging.Logger
    swerve_drive: SwerveDrive  # The swerve drive to test
    test_groups: list[TestGroup]

    _chooser = wpilib.SendableChooser()

    @property
    def current_test_group(self) -> TestGroup:
        """The currently active test group"""
        return self.test_groups[self._current_test_group_index]

    @property
    def current_test_group_index(self) -> int:
        """The index of the currently active test group"""
        return self._current_test_group_index

    @current_test_group_index.setter
    def current_test_group_index(self, value: int):
        """Change the active test group and start the first test of the new test group"""
        if value != self._current_test_group_index:
            if value >= len(self.test_groups):  # A sanity check for crazy values
                value = value % len(self.test_groups)

            self._current_test_group_index = value
            self._current_test = 0
            self.start_time = None

    def __init__(self, swerve_drive: SwerveDrive, logger: logging.Logger):
        super().__init__()
        self.swerve_drive = swerve_drive
        self.logger = logger.getChild("SwerveTestDriver")
        self.start_time = None

        self.populate_tests()

    def populate_tests(self):
        # Expected Outcome: Each wheel should rotate in the same direction
        angle_motor_rotation_tests = TestGroup("Angle Motor Rotation",
                                               "Rotate angle motors with a percentage of full power.  Ensure motors rotate in the same direction.",
                                               [TestConfig(1, self.runAngleMotorTest, (ModulePosition.front_left, 0.1)),
                                                TestConfig(1, self.runAngleMotorTest,
                                                           (ModulePosition.front_right, 0.1)),
                                                TestConfig(1, self.runAngleMotorTest, (ModulePosition.back_left, 0.1)),
                                                TestConfig(1, self.runAngleMotorTest,
                                                           (ModulePosition.back_right, 0.1))])

        # Expected Outcome: Each wheel should rotate to align with the robots forward direction
        angle_motor_orientation_tests = TestGroup(
            "Angle Motor Orientation",
            "Each wheel individually should rotate to align with the robots forward direction." +
            "  This is a test of the absolute encoder and the offset of the encoder." +
            "  The offset should be set so that the wheel is aligned with the robot's forward direction when the robot is initialized." +
            "  The wheels should rotate in the following order: front left, front right, back left, back right",
            [TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, 0)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, 0)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, 0)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, 0)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, math.pi)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, math.pi)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, math.pi)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, math.pi)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_left, -math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.front_right, -math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_left, -math.pi / 2)),
             TestConfig(2.25, self.runAngleMotorPIDTest, (ModulePosition.back_right, -math.pi / 2)),
             ])

        quick_angle_orientation_tests = TestGroup(
            "Quick Angle Motor Orientation",
            "Each wheel  should rotate to align with the robots forward direction" +
            "then rotate in 90 degree increments",
            [TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                            ModulePosition.front_right,
                                                            ModulePosition.back_left,
                                                            ModulePosition.back_right], 0)),
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
             ])

        individual_drive_wheel_rotation_tests = TestGroup(
            "Individual wheel drive test",
            "Drive each wheel individually.  This is a test of rotation conversion factors for the drive wheel" +
            "Each wheel should make one full rotation",

            [TestConfig(.5, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                          ModulePosition.front_right,
                                                          ModulePosition.back_left,
                                                          ModulePosition.back_right], math.pi / 2)),
             #  TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_left, 1)),
             #  TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.front_right, 1)),
             TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_left, 1)),
             TestConfig(2, self.runDriveMotorRotationTest, (ModulePosition.back_right, 10)),
             ]
        )

        drive_tests = TestGroup(
            "Basic Drive Tests",
            "Starting with locked wheels, robot should drive forward, backward, left, right, and rotate clockwise and counter clockwise",
            [
                TestConfig(1.25, self.swerve_drive.lock_wheels, ()),
                # Drive forward and backward
                TestConfig(3, self.runDriveTest, (1, 0, 0)),
                TestConfig(3, self.runDriveTest, (-1, 0, 0)),
                # Drive left and right
                TestConfig(3, self.runDriveTest, (0, 0.5, 0)),
                TestConfig(3, self.runDriveTest, (0, -0.5, 0)),
                # Spin using rotation in both directions
                TestConfig(3, self.runDriveTest, (0, 0, 0.5)),
                TestConfig(3, self.runDriveTest, (0, 0, -0.5))]
        )

        default_tests = TestGroup(
            "Default Test",
            "The place to put whatever test you want to run as you edit the code",
            [
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

                TestConfig(1.25, self.runAngleMotorPIDTests, ([ModulePosition.front_left,
                                                               ModulePosition.front_right,
                                                               ModulePosition.back_left,
                                                               ModulePosition.back_right], 0)),

                # Drive using translation in various directions
                TestConfig(6, self.runDriveDistanceTest, (2, 0)),

                # TestConfig(3, self.runDriveTest, (0, 0.5, 0)),
                TestConfig(6, self.runDriveDistanceTest, (-2, 0)),
                # TestConfig(3, self.runDriveTest, (0, -0.5, 0)),

                # Drive using rotation in both directions
                # TestConfig(3, self.runDriveTest, (0, 0, 0.5)),
                # TestConfig(3, self.runDriveTest, (0, 0, -0.5)),
            ]
        )

        self.test_groups = [
            angle_motor_rotation_tests,
            angle_motor_orientation_tests,
            quick_angle_orientation_tests,
            individual_drive_wheel_rotation_tests,
            drive_tests,
            default_tests
        ]

        # Use the generic test group by default
        self.current_test_group_index = 4  # len(self.test_groups) - 1
        self._chooser = create_test_selection_widget("Test Group", self.test_groups)

        self._chooser.onChange(self.on_test_change)

    def testInit(self):
        return

    def on_test_change(self, selected_test_group_index: int | None):
        if isinstance(selected_test_group_index, int):
            self.current_test_group_index = selected_test_group_index

    def testPeriodic(self):
        # Start the test if there is not test running
        if self.start_time is None:
            self.start_time = time.monotonic()

            test_config = self.current_test_group.tests[self._current_test]
            test_config.test(*test_config.args)
            return

        # Check if it is time for the next test
        elapsed_time = time.monotonic() - self.start_time
        if elapsed_time > self.current_test_group.tests[self._current_test].duration:
            self._current_test += 1
            self.start_time = time.monotonic()
            if self._current_test >= len(self.current_test_group.tests):
                self._current_test = 0

            self.stopMotors()

            test_config = self.current_test_group.tests[self._current_test]
            test_config.test(*test_config.args)

    def stopMotors(self):
        """Turn off the motors to prepare for the next test"""
        self.swerve_drive.stop()

    def runDriveMotorTests(self, position: list[ModulePosition], speed):
        """Turn on the drive motors to a specific power.  Ensures drive motors are functional."""
        for module in position:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorTest(self, position: ModulePosition,
                          speed: float):
        """Turn on the drive motor to a specific power.  Ensures drive motors are functional."""
        self.logger.info(f"{position} drive motor should be moving {speed * 100} % power")
        self.swerve_drive.modules[position].drive_motor.set(speed)

    def runDriveMotorRotationTests(self, num_rotations: list[ModulePosition], speed: float):
        """Rotates the drive wheels a specific number of rotations.  Ensures our wheel size, gear ratio and math
           are correct.  This is important for odometry to measure progress across the field."""
        for module in num_rotations:
            self.runDriveMotorTest(module, speed)

    def runDriveMotorRotationTest(self, position: ModulePosition, num_rotations: float):
        """Rotates the drive wheel a specific number of rotations.  Ensures our wheel size, gear ratio and math
           are correct.  This is important for odometry to measure progress across the field."""
        self.logger.info(f"{position} drive motor wheel should rotate {num_rotations} time(s)")
        self.swerve_drive.modules[position].rotate_drive_wheel(num_rotations)

    def runDriveMotorVelocityTest(self, position: ModulePosition, meters_per_sec: float):
        """Turn on the drive motor to a specific speed.  Ensures that drive motors move at the
        expected speed.  This can be hard to precisely measure, but eyeballing or using a stopwatch as it drives across ground is a start."""
        self.logger.info(f"{position} drive motor should be moving {meters_per_sec} m/s")
        self.swerve_drive.modules[position].velocity = meters_per_sec

    def runAngleMotorTests(self, position: list[ModulePosition],
                           power: float):
        """Turn on the angle motors to a specific power.  Ensures angle motors are functional."""
        for module in position:
            self.runAngleMotorTest(module, power)

    def runAngleMotorTest(self, position: ModulePosition,
                          power: float):
        """Turn on the angle motor to a specific power.  Ensures angle motors are functional."""
        self.logger.info(f"{position} angle motor should be spinning at {power * 100} % power")
        self.swerve_drive.modules[position].angle_motor.set(power)

    def runAngleMotorPIDTests(self, position: list[ModulePosition],
                              angle: float):
        """Set the angles of the motors, does not pass through optimization.  Ensures feedback sensors and pid are correctly set."""
        for module in position:
            self.runAngleMotorPIDTest(module, angle)

    def runAngleMotorPIDTest(self, position: ModulePosition,
                             angle: float):
        """Set the angle of the motor, does not pass through optimization.   Ensures feedback sensors and pid are correctly set."""
        self.logger.info(f"{position} angle motor should move to {math.degrees(angle)} degrees")
        self.swerve_drive.modules[position].angle = angle

    def runDriveTest(self, vx, vy, rotation, run_modules: Sequence[ModulePosition] | None = None):
        """
        Set a desired drive state.  Output should be optimized to use the minimal angle

        :param run_modules: A set of modules to drive.  If None, all modules will be driven.  This is useful for testing individual modules and ensuring ModulePosition is correct for each module
        """
        if run_modules is None:
            self.logger.info(f"Chassis should be driving at {vx}x  {vy}y {rotation} rotation rate")
        else:
            self.logger.info(f"The modules below should be driving at {vx}x  {vy}y {rotation} rotation rate:")
            for m in run_modules:
                self.logger.info(f"\t{m}")

        self.swerve_drive.drive(vx, vy, rotation, run_modules)

        # TODO: Test the drive states to ensure the angle and direction of each wheel is correct

    def runDriveDistanceTest(self, meters: float, angle: float):
        """Drive the robot a specific distance in a specific direction"""
        self.logger.info(f"Drive the robot {meters} meters at {math.degrees(angle)} degrees")
        self.swerve_drive.drive_set_distance(meters, angle)
