import sys
import wpilib
import wpilib.event
import commands.shoot
import config
import hardware
import subsystems
from config import AxisConfig
import swerve
import telemetry
import navx
import drivers
from drivers import TestDriver, TwinStickTeleopDrive, TeleopDrive
from swerve import SwerveDrive
from debug import attach_debugger
from wpilib import SmartDashboard
from wpilib import SmartDashboard as sd
import commands2
import commands2.button
from math_help import Range

from wpimath.controller import ProfiledPIDControllerRadians, ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from computervision.fieldpositioning import AprilTagDetector
import math

######################################################################
# Change the name of the robot here to choose between different robots
from robots import crescendo as robot_config

is_test = False

######################################################################

if __debug__ and "run" in sys.argv:
    # To enter debug mode, add the --debug flag to the 'deploy' command:
    # python -m robotpy deploy --debug
    # At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger()


def create_twinstick_tracking_command(controller: commands2.button.CommandGenericHID,
                                      swerve_drive: swerve.SwerveDrive,
                                      heading_control: subsystems.ChassisHeadingControl):
    return commands.drive.Drive(
        swerve_drive,
        get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                        robot_config.standard_joystick_drive_axis_config),
        get_y=lambda: drivers.map_input(lambda: controller.getRawAxis(0),
                                        robot_config.standard_joystick_drive_axis_config),
        get_theta=lambda: heading_control.desired_velocity)


def create_3dof_command(controller: commands2.button.CommandGenericHID,
                        swerve_drive: swerve.SwerveDrive):
    return commands.drive.Drive(
        swerve_drive,
        get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                        robot_config.standard_joystick_drive_axis_config),
        get_y=lambda: drivers.map_input(lambda: controller.getRawAxis(0),
                                        robot_config.standard_joystick_drive_axis_config),
        get_theta=lambda: drivers.map_input(lambda: controller.getRawAxis(2),
                                            robot_config.standard_joystick_drive_axis_config)
    )


def create_twinstick_heading_command(controller: commands2.button.CommandJoystick,
                                     heading_control: subsystems.ChassisHeadingControl):
    return commands.drive.TwinstickHeadingSetter(
        set_heading_goal=heading_control.setGoal,
        get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                        robot_config.standard_joystick_drive_axis_config),
        get_y=lambda: drivers.map_input(lambda: controller.getRawAxis(0),
                                        robot_config.standard_joystick_drive_axis_config))

class MyRobot(commands2.TimedCommandRobot):
    _command_scheduler: commands2.CommandScheduler

    swerve_drive: SwerveDrive
    swerve_telemetry: telemetry.SwerveTelemetry
    test_driver: TestDriver
    teleop_drive: TeleopDrive
    twinstick_teleop_drive: TwinStickTeleopDrive
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: commands2.button.CommandXboxController
    joystick_one: commands2.button.CommandJoystick
    joystick_two: commands2.button.CommandJoystick

    field: wpilib.Field2d
    april_tag_one: AprilTagDetector

    trapezoid_profile: TrapezoidProfile.Constraints
    rotation_pid: ProfiledPIDControllerRadians

    shooter: subsystems.Shooter
    intake: subsystems.Intake
    indexer: subsystems.Indexer

    _heading_control: subsystems.ChassisHeadingControl


    def __init__(self, period: float = commands2.TimedCommandRobot.kDefaultPeriod / 1000):
        super().__init__(period)

    @property
    def navx(self) -> navx.AHRS:
        return self._navx

    def update_test_mode(self):
        """Sets a global variable indicating that the robot is in test mode"""
        global is_test
        is_test = self.isTest()

    def robotInit(self):
        super().robotInit()
        self.update_test_mode()
        self._command_scheduler = commands2.CommandScheduler()
        self.field = wpilib.Field2d()
        if robot_config.physical_properties.gyro_on_spi:
            self._navx = navx.AHRS.create_spi()
        else:
            self._navx = navx.AHRS.create_i2c()
        self.controller = commands2.button.CommandXboxController(0)
        self.joystick_one = commands2.button.CommandJoystick(0)
        self.joystick_two = commands2.button.CommandJoystick(1)
        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules,
                                               robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)
        SmartDashboard.putData("Field", self.field)
        self.swerve_drive.initialize()
        self.april_tag_one = AprilTagDetector(self.swerve_drive, self.logger)

        self._heading_control = subsystems.ChassisHeadingControl(
            get_chassis_angle_velocity_measurement=lambda: self.swerve_drive.measured_chassis_speed.omega_dps,
            get_chassis_angle_measurement=lambda: self.swerve_drive.gyro_angle_radians,
            angle_pid_config=robot_config.default_heading_pid,
            feedforward_config=robot_config.default_heading_feedforward,
            initial_angle=self.swerve_drive.gyro_angle_radians
        )

        self.test_driver = TestDriver(self.swerve_drive, self.logger)
        # self._command_scheduler.setDefaultCommand(subsystem=self.swerve_drive, defaultCommand=drive_command)

        # self.teleop_drive = TeleopDrive(self.swerve_drive,
        #                                 AxisConfig(deadband=robot_config.joystick_controls.x_deadband,
        #                                            output_range=Range(0,
        #                                                               robot_config.physical_properties.max_drive_speed),
        #                                            controller=self.joystick_one,
        #                                            axis_index=1),
        #                                 AxisConfig(deadband=robot_config.joystick_controls.y_deadband,
        #                                            output_range=Range(0,
        #                                                               robot_config.physical_properties.max_drive_speed),
        #                                            controller=self.joystick_one,
        #                                            axis_index=0),
        #                                 AxisConfig(deadband=robot_config.joystick_controls.theta_deadband,
        #                                            output_range=Range(0,
        #                                                               robot_config.physical_properties.max_rotation_speed),
        #                                            controller=self.joystick_two,
        #                                            axis_index=0))
        # # for standard double flight sticks
        # self.twinstick_teleop_drive = TwinStickTeleopDrive(self.swerve_drive,
        #                                                    AxisConfig(
        #                                                        deadband=robot_config.joystick_controls.x_deadband,
        #                                                        output_range=Range(0,
        #                                                                           robot_config.physical_properties.max_drive_speed),
        #                                                        controller=self.joystick_one,
        #                                                        axis_index=1),
        #                                                    AxisConfig(
        #                                                        deadband=robot_config.joystick_controls.y_deadband,
        #                                                        output_range=Range(0,
        #                                                                           robot_config.physical_properties.max_drive_speed),
        #                                                        controller=self.joystick_one,
        #                                                        axis_index=0),
        #                                                    AxisConfig(
        #                                                        deadband=robot_config.joystick_controls.theta_deadband,
        #                                                        output_range=Range(0,
        #                                                                           robot_config.physical_properties.max_rotation_speed),
        #                                                        controller=self.joystick_two,
        #                                                        axis_index=1),
        #                                                    AxisConfig(
        #                                                        deadband=robot_config.joystick_controls.theta_deadband,
        #                                                        output_range=Range(0,
        #                                                                           robot_config.physical_properties.max_rotation_speed),
        #                                                        controller=self.joystick_two,
        #                                                        axis_index=0),
        #                                                    self.rotation_pid)

        self.shooter = subsystems.Shooter(robot_config.shooter_config, robot_config.default_flywheel_pid, self.logger)
        self.indexer = subsystems.Indexer(robot_config.indexer_config, self.logger)
        self.intake = subsystems.Intake(robot_config.intake_config, self.logger)

    def robotPeriodic(self) -> None:
        super().robotPeriodic()  # This calls the periodic functions of the subsystems
        self.swerve_drive.periodic()
        self.april_tag_one.periodic()
        self.field.setRobotPose(self.swerve_drive.pose)
        sd.putData("Field", self.field)
        self.swerve_telemetry.report_to_dashboard()

    def teleopInit(self):
        driving_command = create_twinstick_tracking_command(self.joystick_one,
                                                            self.swerve_drive,
                                                            self._heading_control)
        heading_command = create_twinstick_heading_command(self.joystick_two,
                                                           self._heading_control)
        self._command_scheduler.schedule(heading_command, driving_command)

    def teleopPeriodic(self):
        super().teleopPeriodic()
        # self.twinstick_teleop_drive.drive()
        #  self.teleop_drive.drive()
        self.joystick_two.button(1).toggleOnTrue(commands.Load(self.intake, self.indexer))
        self.joystick_one.button(1).toggleOnTrue(commands.Shoot(self.shooter, self.indexer))
        # if self.joystick_one.getRawButton(1) and not self.button_state_zero:
        #     self.button_state_zero = self.joystick_one.getRawButton(1)
        #     shoot = commands.Shoot(self.shooter, self.indexer)
        #     self._command_scheduler.getInstance().schedule(shoot)
        #
        # if self.joystick_one.getRawButton(2) and not self.button_state_one:
        #     self.button_state_one = self.joystick_one.getRawButton(2)
        #     load = commands.Load(self.intake, self.indexer)
        #     self._command_scheduler.getInstance().schedule(load)

    #         shoot_event_loop = wpilib.event.EventLoop()
    #         shoot_event_loop.bind(lambda: commands.shoot.Shoot(self.shooter, self.indexer))
    #         commands2.button.trigger.Trigger(shoot_event_loop,
    #                                          lambda: self.joystick_one.button(0))

    def updateField(self):
        pass

    def autonomousInit(self):
        super().autonomousInit()

    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()
        self.test_driver.testInit()

    def testPeriodic(self) -> None:
        super().testPeriodic()
        self.test_driver.testPeriodic()
