import sys

import robotpy_apriltag
import wpilib
import wpilib.event
import commands
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
from wpilib import DriverStation

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


def get_alliance_adjusted_axis(controller: commands2.button.CommandGenericHID, i_axis: int) -> float:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        return -controller.getRawAxis(i_axis)
    else:
        return controller.getRawAxis(i_axis)


def create_twinstick_tracking_command(controller: commands2.button.CommandGenericHID,
                                      swerve_drive: swerve.SwerveDrive,
                                      heading_control: subsystems.ChassisHeadingControl):
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        x_axis_func = lambda: drivers.map_input(lambda: -controller.getRawAxis(1),
                                                robot_config.standard_joystick_drive_axis_config)
    else:
        x_axis_func = lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                                robot_config.standard_joystick_drive_axis_config)

    return commands.drive.Drive(
        swerve_drive,
        get_x=x_axis_func,
        # get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
        #                                 robot_config.standard_joystick_drive_axis_config),
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


def create_twinstick_heading_command(controller: commands2.button.CommandGenericHID,
                                     heading_control: subsystems.ChassisHeadingControl):
    return commands.drive.TwinstickHeadingSetter(
        set_heading_goal=heading_control.setGoal,
        get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                        robot_config.standard_joystick_rotation_axis_config),
        get_y=lambda: drivers.map_input(lambda: controller.getRawAxis(0),
                                        robot_config.standard_joystick_rotation_axis_config))


class MyRobot(commands2.TimedCommandRobot):
    _command_scheduler: commands2.CommandScheduler

    swerve_drive: SwerveDrive
    swerve_telemetry: telemetry.SwerveTelemetry
    heading_controller_telemetry: telemetry.ChassisHeadingTelemetry
    test_driver: TestDriver
    teleop_drive: TeleopDrive
    twinstick_teleop_drive: TwinStickTeleopDrive
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: commands2.button.CommandXboxController
    joystick_one: commands2.button.CommandJoystick
    joystick_two: commands2.button.CommandJoystick
    operator_control: commands2.button.CommandJoystick

    field: wpilib.Field2d
    april_tag_one: AprilTagDetector

    trapezoid_profile: TrapezoidProfile.Constraints
    rotation_pid: ProfiledPIDControllerRadians

    shooter: subsystems.Shooter
    intake: subsystems.Intake
    indexer: subsystems.Indexer

    _heading_control: subsystems.ChassisHeadingControl
    _x_axis_control: subsystems.AxisPositionControl
    _y_axis_control: subsystems.AxisPositionControl

    apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout

    robot_control_commands: list

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
        self.apriltagfieldlayout = robotpy_apriltag.loadAprilTagLayoutField(
            robotpy_apriltag.AprilTagField.k2024Crescendo)
        if robot_config.physical_properties.gyro_on_spi:
            self._navx = navx.AHRS.create_spi()
        else:
            self._navx = navx.AHRS.create_i2c()

        self.controller = commands2.button.CommandXboxController(0)
        self.joystick_one = commands2.button.CommandJoystick(0)
        self.joystick_two = commands2.button.CommandJoystick(1)
        self.operator_control = commands2.button.CommandJoystick(2)

        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules,
                                               robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)
        SmartDashboard.putData("Field", self.field)
        self.swerve_drive.initialize()
        self.april_tag_one = AprilTagDetector(self.swerve_drive, self.logger)

        self.init_positioning_pids()

        self._heading_control.enable()
        self._x_axis_control.enable()
        self._y_axis_control.enable()

        self.heading_controller_telemetry = telemetry.ChassisHeadingTelemetry(self._heading_control)
        self.test_driver = TestDriver(self.swerve_drive, self.logger)

        self.shooter = subsystems.Shooter(robot_config.shooter_config, robot_config.default_flywheel_pid, self.logger)
        self.indexer = subsystems.Indexer(robot_config.indexer_config, self.logger)
        self.intake = subsystems.Intake(robot_config.intake_config, self.logger)

        self.joystick_one.button(1).toggleOnTrue(commands.Load(self.intake, self.indexer))
        self.joystick_two.button(1).toggleOnTrue(commands.Shoot(self.shooter, self.indexer))
        self.joystick_one.button(3).toggleOnTrue(commands.SpinupShooter(self.shooter))

        self.operator_control.button(1).toggleOnTrue(commands.Load(self.intake, self.indexer))
        self.operator_control.button(2).toggleOnTrue(commands.Shoot(self.shooter, self.indexer))


        # POINTING COMMANDS USING LOCATION FOR RED ON JOYSTICK 2
        ################################################################################################################
        ###POINT TOWARDS RED SPEAKER ON BUTTON 3 -> STICK 2
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=4,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_two.button(3).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS RED AMP ON BUTTON 4 -> STICK 2
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=5,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_two.button(4).toggleOnTrue(april_tag_pointer)
        #
        # # STAGE POINTING COMMANDS FOR RED
        #
        # ###POINT TOWARDS RED STAGE SOURCE SIDE ON BUTTON 6 -> STICK 2
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=11,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_two.button(6).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS RED STAGE AMP SIDE ON BUTTON 7 -> STICK 2
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=12,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_two.button(7).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS RED STAGE FAR SIDE ON BUTTON 8 -> STICK 2
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=13,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_two.button(8).toggleOnTrue(april_tag_pointer)
        # ################################################################################################################
        #
        # # POINTING COMMANDS USING LOCATION FOR BLUE JOYSTICK 1
        # ################################################################################################################
        # ###POINT TOWARDS BLUE SPEAKER ON BUTTON 6 -> STICK 1
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=7,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_one.button(6).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS BLUE AMP ON BUTTON 7 -> STICK 1
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=6,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_one.button(7).toggleOnTrue(april_tag_pointer)
        #
        # # STAGE POINTING COMMANDS FOR RED
        #
        # ###POINT TOWARDS BLUE STAGE SOURCE SIDE ON BUTTON 8 -> STICK 1
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=16,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_one.button(8).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS BLUE STAGE AMP SIDE ON BUTTON 9 -> STICK 1
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=15,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_one.button(9).toggleOnTrue(april_tag_pointer)
        #
        # ###POINT TOWARDS BLUE STAGE FAR SIDE ON BUTTON 10 -> STICK 1
        # april_tag_pointer = commands.AprilTagPointer(set_heading_goal=self._heading_control.setTarget,
        #                                              aprilTagNumber=14,
        #                                              apriltagfieldlayout=self.apriltagfieldlayout,
        #                                              get_xy=lambda: (
        #                                                  self.swerve_drive.pose.x, self.swerve_drive.pose.y))
        # april_tag_pointer.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}
        # self.joystick_one.button(10).toggleOnTrue(april_tag_pointer)
        # ################################################################################################################

        self.driving_command = create_twinstick_tracking_command(self.joystick_one,
                                                                 self.swerve_drive,
                                                                 self._heading_control)
        self.heading_command = create_twinstick_heading_command(self.joystick_two,
                                                                self._heading_control)

        # RETURN COMMAND TO JOYSTICK BUTTON 2
        self.joystick_one.button(2).toggleOnTrue(self.heading_command)
        self.heading_command.requirements = {subsystems.chassis_heading_control.ChassisHeadingControl}

    def init_positioning_pids(self):
        self._heading_control = subsystems.ChassisHeadingControl(
            get_chassis_angle_velocity_measurement=lambda: math.radians(
                self.swerve_drive.measured_chassis_speed.omega_dps),
            get_chassis_angle_measurement=lambda: self.swerve_drive.gyro_angle_radians,
            angle_pid_config=robot_config.default_heading_pid,
            feedforward_config=None,
            initial_angle=self.swerve_drive.gyro_angle_radians
        )

        # TODO: update intial position again after photonvision
        self._x_axis_control = subsystems.AxisPositionControl(
            get_chassis_position_measurement=lambda: self.swerve_drive.pose.x,
            get_chassis_velocity_measurement=lambda: self.swerve_drive.measured_chassis_speed.vx,
            pid_config=robot_config.default_axis_pid,
            feedforward_config=None,
            initial_position=self.swerve_drive.odemetry.getEstimatedPosition().x
        )

        self._y_axis_control = subsystems.AxisPositionControl(
            get_chassis_position_measurement=lambda: self.swerve_drive.pose.y,
            get_chassis_velocity_measurement=lambda: self.swerve_drive.measured_chassis_speed.vy,
            pid_config=robot_config.default_axis_pid,
            feedforward_config=None,
            initial_position=self.swerve_drive.odemetry.getEstimatedPosition().y
        )

    #     self.register_subsystems()
    #
    # def register_subsystems(self):
    #     self._command_scheduler.registerSubsystem(self.swerve_drive)
    #     self._command_scheduler.registerSubsystem(self.shooter)
    #     self._command_scheduler.registerSubsystem(self.indexer)
    #     self._command_scheduler.registerSubsystem(self.intake)

    def robotPeriodic(self) -> None:
        super().robotPeriodic()  # This calls the periodic functions of the subsystems
        self.swerve_drive.periodic()
        self.april_tag_one.periodic()
        self.field.setRobotPose(self.swerve_drive.pose)
        sd.putData("Field", self.field)
        self.swerve_telemetry.report_to_dashboard()
        self.heading_controller_telemetry.report_to_dashboard()

    def teleopInit(self):

        driving_command = create_twinstick_tracking_command(self.joystick_one,
                                                            self.swerve_drive,
                                                            self._heading_control)
        heading_command = create_twinstick_heading_command(self.joystick_two,
                                                           self._heading_control)
        three_dof_command = create_3dof_command(self.joystick_one,
                                                self.swerve_drive)
        # self._command_scheduler.cancel(cmd)
        self._command_scheduler.schedule(heading_command)
        self._command_scheduler.schedule(driving_command)

    def teleopPeriodic(self):
        super().teleopPeriodic()

    def updateField(self):
        pass

    def autonomousInit(self):
        super().autonomousInit()

        estimated_pose = self.swerve_drive.odemetry.getEstimatedPosition()
        self._x_axis_control.set_current_position(estimated_pose.x)
        self._y_axis_control.set_current_position(estimated_pose.y)

        cmd = commands2.cmd.ParallelRaceGroup(
            commands2.cmd.SequentialCommandGroup(
                commands.Shoot(self.shooter, self.indexer),
                commands2.cmd.ParallelCommandGroup(
                    commands.Load(self.intake, self.indexer),
                    commands.DeadReckonX(self.swerve_drive, .5)
                ),
            ),
            commands2.cmd.WaitCommand(15)
        )

        # self._command_scheduler.schedule(cmd)


    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()
        self.test_driver.testInit()

    def testPeriodic(self) -> None:
        super().testPeriodic()
        self.test_driver.testPeriodic()
