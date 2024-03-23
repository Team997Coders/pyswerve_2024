import sys
import autos
import wpilib
import wpilib.event
import commands
import config
import subsystems
import swerve
import telemetry
import navx
import drivers
from drivers import TestDriver, TwinStickTeleopDrive, TeleopDrive
from swerve import SwerveDrive
from wpilib import SmartDashboard as sd
import commands2
import commands2.button
from wpilib import DriverStation
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfile
import math
# from pathplannerlib.auto import PathPlannerAuto
# from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.auto import NamedCommands
# from pathplannerlib.path import PathPlannerPath
# import pathplannerlib.auto

# import robotpy_apriltag
# from subsystems.photonvision import PhotonVisionAprilTagDetector


######################################################################
# Change the name of the robot here to choose between different robots
from robots import crescendo as robot_config

is_test = True
######################################################################

# To run this code:
# windows: python -m robotpy deploy
# mac: python3 -m robotpy deploy --skip-test


def get_alliance_adjusted_axis(controller: commands2.button.CommandGenericHID, i_axis: int) -> float:
    if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
        return -controller.getRawAxis(i_axis)
    else:
        return controller.getRawAxis(i_axis)


def create_twinstick_tracking_command(controller: commands2.button.CommandGenericHID,
                                      swerve_drive: swerve.SwerveDrive,
                                      heading_control: subsystems.ChassisHeadingControl):
    return commands.drive.Drive(
        swerve_drive,
        get_x=lambda: drivers.map_input(lambda: controller.getRawAxis(1),
                                        robot_config.standard_joystick_drive_axis_config),
        get_y=lambda: drivers.map_input(lambda: controller.getRawAxis(0),
                                        robot_config.standard_joystick_drive_axis_config),
        get_theta=lambda: heading_control.desired_velocity
    )


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
                                        robot_config.standard_joystick_rotation_axis_config),
        is_heading_inverted=False)


class MyRobot(commands2.TimedCommandRobot):
    _command_scheduler: commands2.CommandScheduler

    heading_controller_telemetry: telemetry.ChassisHeadingTelemetry
    x_axis_telemetry: telemetry.AxisPositionTelemetry
    y_axis_telemetry: telemetry.AxisPositionTelemetry

    shooter_telemetry: telemetry.ShooterTelemetry | None = None
    indexer_telemetry: telemetry.IndexerTelemetry | None = None
    intake_telemetry: telemetry.IntakeTelemetry | None = None
    climber_telemetry: telemetry.ClimberTelemetry | None = None
    swerve_telemetry: telemetry.SwerveTelemetry

    test_driver: TestDriver
    teleop_drive: TeleopDrive
    twinstick_teleop_drive: TwinStickTeleopDrive
    _navx: navx.AHRS  # Attitude Heading Reference System

    joystick_one: commands2.button.CommandJoystick
    joystick_two: commands2.button.CommandJoystick
    operator_control: commands2.button.CommandXboxController

    #field: wpilib.Field2d
    #april_tag_one: PhotonVisionAprilTagDetector | None = None
    limelight_positioning: subsystems.LimeLightPositioning | None = None

    trapezoid_profile: TrapezoidProfile.Constraints
    rotation_pid: ProfiledPIDControllerRadians

    shooter: subsystems.Shooter | None = None
    intake: subsystems.Intake | None = None
    indexer: subsystems.Indexer | None = None
    climber: subsystems.Climber | None = None
    swerve_drive: SwerveDrive

    _heading_control: subsystems.ChassisHeadingControl
    _x_axis_control: subsystems.AxisPositionControl
    _y_axis_control: subsystems.AxisPositionControl
    _target_heading_mappings: dict[tuple[commands2.button.CommandGenericHID, int], tuple[float, float]]
    #apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    auto_options: list[autos.AutoFactory]
    robot_control_commands: list
    auto_chooser: wpilib.SendableChooser
    sysid: subsystems.swerve_system_id

    def __init__(self, period: float = commands2.TimedCommandRobot.kDefaultPeriod / 1000):
        super().__init__(period)
        self.config = robot_config

    def update_test_mode(self):
        """Sets a global variable indicating that the robot is in test mode"""
        global is_test
        is_test = self.isTest()

    def robotInit(self):
        super().robotInit()
        self.update_test_mode()
        self._command_scheduler = commands2.CommandScheduler()
        #self.apriltagfieldlayout = robotpy_apriltag.loadAprilTagLayoutField(
        #    robotpy_apriltag.AprilTagField.k2024Crescendo)
        if robot_config.physical_properties.gyro_on_spi:
            self._navx = navx.AHRS.create_spi()
        else:
            self._navx = navx.AHRS.create_i2c()

        self.joystick_one = commands2.button.CommandJoystick(0)
        self.joystick_two = commands2.button.CommandJoystick(1)
        self.operator_control = commands2.button.CommandXboxController(2)

        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules,
                                               robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)
        self.swerve_drive.initialize()
        # self.limelight_positioning = subsystems.LimeLightPositioning(self.swerve_drive,
        #                                                              robot_config.limelight_camera_config,
        #                                                              self.logger)
        self.init_positioning_pids()
        self.init_position_control_telemetry()
        self._heading_control.enable()
        self._x_axis_control.enable()
        self._y_axis_control.enable()
        self.sysid = subsystems.swerve_system_id(self.swerve_drive, "swerve")
        self.heading_controller_telemetry = telemetry.ChassisHeadingTelemetry(self._heading_control)
        self.test_driver = TestDriver(self.swerve_drive, self.logger)
        self.driving_command = create_twinstick_tracking_command(self.joystick_one,
                                                                 self.swerve_drive,
                                                                 self._heading_control)
        self.heading_command = create_twinstick_heading_command(self.joystick_two,
                                                                self._heading_control)
        self.driving_command.requirements = {self.swerve_drive}
        self.heading_command.requirements = {self._heading_control}
        self.try_init_mechanisms()
        self.define_autonomous_modes()
        self.auto_chooser = telemetry.create_selector("Autos", [auto.name for auto in self.auto_options])

    def define_autonomous_modes(self):

        self.auto_options = [
            autos.AutoFactory("Drive Forward and backward", autos.auto_calibrations.create_drive_forward_and_back_auto,
                              (self.swerve_drive, self._x_axis_control, self._y_axis_control, self._heading_control)),
            autos.AutoFactory("SysId: Dynamic", self.sysid.create_dynamic_measurement_command, ()),
            autos.AutoFactory("SysId: Quasistatic", self.sysid.create_quasistatic_measurement_command, ()),
            autos.AutoFactory("One note auto", autos.manual_autos.one_note_auto(self), (self)),
            # autos.AutoFactory("Two Note auto>", autos.manual_autos.two_note_auto(self), (self)),
            # autos.AutoFactory("three Note auto>", autos.manual_autos.three_note_auto(self), (self)),
            # autos.AutoFactory("Taxi", autos.manual_autos.taxi(self), (self))
            ]

        if robot_config.has_mechanisms:
            self.auto_options.append(
                autos.AutoFactory("One Note auto", autos.manual_autos.one_note_auto,
                                  (self,))
            # self.auto_options.append(
            #     autos.AutoFactory("Three Note auto", autos.manual_autos.three_note_auto,
            #                       (self,)))
            # self.auto_options.append(
            #     autos.AutoFactory("Taxi", autos.manual_autos.one_note_auto,
            # #                       (self,)))
            # self.auto_options.append(
            #     autos.AutoFactory("Taxi", autos.manual_autos.taxi,
            #                       (self))
            )

    def try_init_mechanisms(self):
        """Initialize mechanisms if they are present in the robot config"""
        sd.putBoolean("Has Mechanisms", robot_config.has_mechanisms)
        if robot_config.has_mechanisms:
            self.shooter = subsystems.Shooter(robot_config.shooter_config, robot_config.default_flywheel_pid,
                                              self.logger)
            self.indexer = subsystems.Indexer(robot_config.indexer_config, self.logger)
            self.intake = subsystems.Intake(robot_config.intake_config, self.logger)
            self.climber = subsystems.Climber(robot_config.climber_config, self.logger)
            self.init_mechanism_telemetry()

# left joystick
            self.joystick_one.button(1).toggleOnTrue(commands.Load(self.intake, self.indexer))  # Intake
            self.joystick_one.button(2).toggleOnTrue(commands.Outtake(self.intake, self.indexer))  # Outtake
# right joystick
            self.joystick_two.button(1).whileTrue(commands.Shoot(self.shooter, self.indexer))  # Shoot
# operator xbox controller
            self.operator_control.leftBumper().onTrue(commands.ClimberUp(self.climber)).onFalse(commands.ClimberStop(self.climber))  # climber up
            self.operator_control.rightBumper().onTrue(commands.ClimberDown(self.climber)).onFalse(commands.ClimberStop(self.climber))  # climber down

    def init_mechanism_telemetry(self):
        if robot_config.has_mechanisms:
            telemetry.mechanisms_telemetry.ShowMechansimPIDs(self)
            self.intake_telemetry = telemetry.IntakeTelemetry(self.intake.config)
            self.indexer_telemetry = telemetry.IndexerTelemetry(self.indexer)
            self.shooter_telemetry = telemetry.ShooterTelemetry(self.shooter.config)
            self.climber_telemetry = telemetry.ClimberTelemetry(self.climber.config)

    def mechanism_telemetry_periodic(self):
        if robot_config.has_mechanisms:
            self.intake_telemetry.periodic()
            self.indexer_telemetry.periodic()
            self.shooter_telemetry.periodic()
            self.climber_telemetry.periodic()

    def init_positioning_pids(self):
        self._heading_control = subsystems.ChassisHeadingControl(
            get_chassis_angle_velocity_measurement=lambda: math.radians(
                self.swerve_drive.measured_chassis_speed.omega_dps),
            get_chassis_angle_measurement=lambda: self.swerve_drive.gyro_angle_radians,
            angle_pid_config=robot_config.default_heading_pid,
            feedforward_config=None,
            initial_angle=self.swerve_drive.gyro_angle_radians
        )

        # TODO: update intial position again after photonvision.  This is currently done in AutonoumousInit,
        # but there may be a better way to do this.  Do we check in every initializer?
        self._x_axis_control = subsystems.AxisPositionControl(
            get_chassis_position_measurement=lambda: self.swerve_drive.pose.x,
            get_chassis_velocity_measurement=lambda: self.swerve_drive.measured_chassis_speed.vx,
            pid_config=robot_config.default_axis_pid,
            feedforward_config=None,
            initial_position=self.swerve_drive.estimated_position.x
        )

        self._y_axis_control = subsystems.AxisPositionControl(
            get_chassis_position_measurement=lambda: self.swerve_drive.pose.y,
            get_chassis_velocity_measurement=lambda: self.swerve_drive.measured_chassis_speed.vy,
            pid_config=robot_config.default_axis_pid,
            feedforward_config=None,
            initial_position=self.swerve_drive.estimated_position.y
        )

    def init_position_control_telemetry(self):
        self.x_axis_telemetry = telemetry.AxisPositionTelemetry("X", self._x_axis_control)
        self.y_axis_telemetry = telemetry.AxisPositionTelemetry("Y", self._y_axis_control)
        self.heading_controller_telemetry = telemetry.ChassisHeadingTelemetry(self._heading_control)

    def robotPeriodic(self) -> None:
        super().robotPeriodic()  # This calls the periodic functions of the subsystems
        self.mechanism_telemetry_periodic()
        sd.putBoolean("NavX Connected?", self._navx.isConnected())

    def disabledInit(self):
        super().disabledInit()
        self._command_scheduler.schedule(
            commands2.cmd.ParallelCommandGroup(
                commands.IndexOff(self.indexer),
                commands.SpindownShooter(self.shooter),
                commands.IntakeOff(self.intake),
                commands.ClimberStop(self.climber),
            )
        )
        self._command_scheduler.cancelAll()
    def teleopInit(self):
        # driving_command = create_twinstick_tracking_command(self.joystick_one,
        #                                                     self.swerve_drive,
        #                                                     self._heading_control)
        # heading_command = create_twinstick_heading_command(self.joystick_two,
        #                                                    self._heading_control)
        # three_dof_command = create_3dof_command(self.joystick_one,
        #                                         self.swerve_drive)
        self._command_scheduler.schedule(self.heading_command)
        self._command_scheduler.schedule(self.driving_command)


    def reset_pose_pids_to_current_position(self):
        """Sets the current position of the driving pids to the estimated position of the robot"""
        estimated_pose = self.swerve_drive.estimated_position
        self._x_axis_control.set_current_position(estimated_pose.x)
        self._y_axis_control.set_current_position(estimated_pose.y)
        self._heading_control.set_current_position(self.swerve_drive.gyro_angle_radians)


    def autonomousInit(self):
        super().autonomousInit()
        print("Auto Init")
        #  Hopefully at this point we've gotten an april tag fix.  Use that
        #  information to update our positioning pids
        self.reset_pose_pids_to_current_position()
        auto_path_index = self.auto_chooser.getSelected()
        factory = self.auto_options[auto_path_index]
        sd.putString("Selected auto", factory.name)
        cmds = factory.create(*factory.args)
        # Factories can return either a set of commands or a single command.  Call the scheduler accordingly
        if isinstance(cmds, commands2.Command):
            self._command_scheduler.schedule(cmds)
        else:
            self._command_scheduler.schedule(*cmds)

    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()
        # self.test_driver.testInit()
        # self._command_scheduler.schedule(commands.TestMechanisms(
        #     indexer=self.indexer, intake=self.intake, shooter=self.shooter, climber=self.climber))

    def testPeriodic(self) -> None:
        super().testPeriodic()
        # self.test_driver.testPeriodic()
