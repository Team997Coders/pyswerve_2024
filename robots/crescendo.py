import math_help
import math
from config import PIDConfig, DriverControlsConfig, MotorConfig, ModulePosition, SwerveModuleConfig, EncoderConfig, \
    OptionalRange, PhysicalConfig, OptionalSwerveModuleIntProperty, SwerveModuleFloatProperty, \
    OptionalSwerveModuleFloatProperty, ShooterConfig, IndexerConfig, IntakeConfig, ClimberConfig, \
    ProfiledPIDConfig, VelocityAccelerationConfig, PositionVelocityConfig, FeedForwardConfig, SwerveModuleIntProperty, \
    AxisConfig

from .shared import swerve_current_limit, swerve_ramp_rate

#Panel is #13

default_angle_pid = PIDConfig(p=.6, i=0.0, d=0.2, wrapping=OptionalRange(min=0, max=math.pi * 2), tolerance=None)
# Be Careful when adding an i value to the drive pid, it can cause the robot to drive very fast
default_drive_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None, tolerance=None)
default_heading_pid = ProfiledPIDConfig(p=.18, i=0.12, d=0.001,
                                         wrapping=OptionalRange(min=-math.pi, max=math.pi),
                                         profile=VelocityAccelerationConfig(velocity=math.pi * 4, acceleration=math.pi),
                                         tolerance=PositionVelocityConfig(position=math.pi / 180, velocity=0.05)
                                         )
default_heading_feedforward = FeedForwardConfig(kS=0.0,
                                                kV=0.01,
                                                kA=0.001)
default_flywheel_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None, tolerance=None)

joystick_controls = DriverControlsConfig(x_deadband=math_help.Range(0.15, 1),
                                         y_deadband=math_help.Range(0.15, 1),
                                         theta_deadband=math_help.Range(0.5, 1))
gamepad_controls = DriverControlsConfig(x_deadband=math_help.Range(0.10, 1),
                                        y_deadband=math_help.Range(0.10, 1),
                                        theta_deadband=math_help.Range(0.10, 1))

shooter_config = ShooterConfig(left_motor=MotorConfig(id=11, inverted=False),
                               right_motor=MotorConfig(id=12, inverted=True),
                               right_flywheel_gear_ratio=1,
                               left_flywheel_gear_ratio=1,
                               right_flywheel_diameter_cm=12,
                               left_flywheel_diameter_cm=12,
                               default_velocity=3,
                               default_fire_time=1,
                               default_spinup_delay=1)  # add motor configs
indexer_config = IndexerConfig(MotorConfig(id=10, inverted=False), indexer_sensor_id=0, indexer_sensor_inverted=True,
                               pid=PIDConfig(p=.000001, i=0, d=0, wrapping=None, tolerance=None),
                               default_velocity=300)  # fix feeder_sensor_id
intake_config = IntakeConfig(MotorConfig(id=9, inverted=True), pid=PIDConfig(p=.000001, i=0, d=0, wrapping=None),
                             default_velocity=.5)
climber_config = ClimberConfig(MotorConfig(id=13, inverted=False), climber_pid=PIDConfig(p=.2, i=0, d=0, wrapping=None))

physical_properties = PhysicalConfig(wheel_diameter_cm=12,
                                     wheel_grip_coefficient_of_friction=1,
                                     encoder_pulses_per_revolution=SwerveModuleFloatProperty(drive=1, angle=1),
                                     gear_ratio=SwerveModuleFloatProperty(angle=150.0 / 7, drive=6.75),
                                     max_drive_speed=3,
                                     max_rotation_speed=math.pi / 6,
                                     fw_set_retries=5,
                                     fw_set_retry_delay_sec=0.05,
                                     invert_gyro=False,
                                     gyro_on_spi=True)



swerve_modules = {ModulePosition.front_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=8, inverted=False,
                                                                 open_ramp_rate=swerve_ramp_rate.drive,
                                                                 closed_ramp_rate=swerve_ramp_rate.drive,
                                                                 current_limit=swerve_current_limit.drive),
                                         angle_motor=MotorConfig(id=1, inverted=True,
                                                                 open_ramp_rate=swerve_ramp_rate.angle,
                                                                 closed_ramp_rate=swerve_ramp_rate.angle,
                                                                 current_limit=swerve_current_limit.angle),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                               inverted=False),
                                         location=(10.375, 10.375),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.front_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=6, inverted=False,
                                                                 open_ramp_rate=swerve_ramp_rate.drive,
                                                                 closed_ramp_rate=swerve_ramp_rate.drive,
                                                                 current_limit=swerve_current_limit.drive),
                                         angle_motor=MotorConfig(id=7, inverted=True,
                                                                 open_ramp_rate=swerve_ramp_rate.angle,
                                                                 closed_ramp_rate=swerve_ramp_rate.angle,
                                                                 current_limit=swerve_current_limit.angle),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                               inverted=False),
                                         location=(10.375, -10.375),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.back_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=4, inverted=False,
                                                                 open_ramp_rate=swerve_ramp_rate.drive,
                                                                 closed_ramp_rate=swerve_ramp_rate.drive,
                                                                 current_limit=swerve_current_limit.drive),
                                         angle_motor=MotorConfig(id=5, inverted=True,
                                                                 open_ramp_rate=swerve_ramp_rate.angle,
                                                                 closed_ramp_rate=swerve_ramp_rate.angle,
                                                                 current_limit=swerve_current_limit.angle),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                               inverted=False),
                                         location=(-10.375, -10.375),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.back_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=2, inverted=False,
                                                                 open_ramp_rate=swerve_ramp_rate.drive,
                                                                 closed_ramp_rate=swerve_ramp_rate.drive,
                                                                 current_limit=swerve_current_limit.drive),
                                         angle_motor=MotorConfig(id=3, inverted=True,
                                                                 open_ramp_rate=swerve_ramp_rate.angle,
                                                                 closed_ramp_rate=swerve_ramp_rate.angle,
                                                                 current_limit=swerve_current_limit.angle),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                               inverted=False),
                                         location=(-10.375, 10.375),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid)

                  }  # type: dict[ModulePosition, SwerveModuleConfig]

standard_joystick_drive_axis_config = AxisConfig(deadband=math_help.Range(0.05, 1),
                                                 output_range=math_help.Range(0, physical_properties.max_drive_speed))

standard_gamepad_drive_axis_config = AxisConfig(deadband=math_help.Range(0.10, 1),
                                                output_range=math_help.Range(0, physical_properties.max_drive_speed))
