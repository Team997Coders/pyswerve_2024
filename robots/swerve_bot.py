import math_help
import math
from config import PIDConfig, DriverControlsConfig, MotorConfig, ModulePosition, SwerveModuleConfig, EncoderConfig, \
    OptionalRange, PhysicalConfig, OptionalSwerveModuleIntProperty, SwerveModuleFloatProperty, \
    OptionalSwerveModuleFloatProperty, ShooterConfig, IndexerConfig, IntakeConfig, ClimberConfig, \
    ProfiledPIDConfig, VelocityAccelerationConfig, PositionVelocityConfig, FeedForwardConfig

joystick_controls = DriverControlsConfig(x_deadband=math_help.Range(0.15, 1),
                                         y_deadband=math_help.Range(0.15, 1),
                                         theta_deadband=math_help.Range(0.05, 1))
gamepad_controls = DriverControlsConfig(x_deadband=math_help.Range(0.10, 1),
                                        y_deadband=math_help.Range(0.10, 1),
                                        theta_deadband=math_help.Range(0.10, 1))

default_angle_pid = PIDConfig(p=.6, i=0.0, d=0.2, wrapping=OptionalRange(min=0, max=math.pi * 2))
# Be Careful when adding an i value to the drive pid, it can cause the robot to drive very fast
default_drive_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None)
default_rotation_pid = ProfiledPIDConfig(p=.18, i=0.12, d=0.001,
                                         wrapping=OptionalRange(min=-math.pi, max=math.pi),
                                         profile=VelocityAccelerationConfig(velocity=math.pi * 4, acceleration=math.pi),
                                         tolerance=PositionVelocityConfig(position=math.pi / 180, velocity=0.05)
                                         )
default_flywheel_pid = PIDConfig(p=0.5, i=0.0, d=0.05, wrapping=None)

shooter_config = ShooterConfig(left_motor=MotorConfig(id=12, inverted=False),
                               right_motor=MotorConfig(id=13, inverted=False),
                               right_flywheel_gear_ratio=1,
                               left_flywheel_gear_ratio=1,
                               right_flywheel_diameter_cm=5,
                               left_flywheel_diameter_cm=5,
                               default_velocity=5,
                               default_fire_time=.3,
                               default_spinup_delay=.2)  # add motor configs
indexer_config = IndexerConfig(MotorConfig(id=10, inverted=True), indexer_sensor_id=14, indexer_sensor_inverted=True,
                               pid=PIDConfig(p=.5, i=0, d=0, wrapping=None), default_velocity=1)
intake_config = IntakeConfig(MotorConfig(id=9, inverted=False), pid=PIDConfig(p=1, i=0, d=0, wrapping=None),
                             default_velocity=1)
climber_config = ClimberConfig(MotorConfig(id=11, inverted=False), climber_pid=PIDConfig(p=.5, i=0, d=0, wrapping=None))

swerve_modules = {ModulePosition.front_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=8, inverted=False),
                                   angle_motor=MotorConfig(id=1, inverted=True),
                                   encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                         inverted=False),
                                   location=(9.5, 9.5),
                                   angle_pid=default_angle_pid,
                                   drive_pid=default_drive_pid),
                  ModulePosition.front_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=6, inverted=False),
                                   angle_motor=MotorConfig(id=7, inverted=True),
                                   encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                         inverted=False),
                                   location=(9.5, -9.5),
                                   angle_pid=default_angle_pid,
                                   drive_pid=default_drive_pid),
                  ModulePosition.back_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=4, inverted=False),
                                   angle_motor=MotorConfig(id=5, inverted=True),
                                   encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                         inverted=False),
                                   location=(-9.5, -9.5),
                                   angle_pid=default_angle_pid,
                                   drive_pid=default_drive_pid),
                  ModulePosition.back_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=2, inverted=False),
                                   angle_motor=MotorConfig(id=3, inverted=True),
                                   encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2,
                                                         inverted=False),
                                   location=(-9.5, 9.5),
                                   angle_pid=default_angle_pid,
                                   drive_pid=default_drive_pid)

                  }  # type: dict[ModulePosition, SwerveModuleConfig]

physical_properties = PhysicalConfig(wheel_diameter_cm=12,
                                     wheel_grip_coefficient_of_friction=1,
                                     current_limit=OptionalSwerveModuleIntProperty(drive=40, angle=20),
                                     ramp_rate=OptionalSwerveModuleFloatProperty(drive=0.25, angle=0.25),
                                     encoder_pulses_per_revolution=SwerveModuleFloatProperty(drive=1, angle=1),
                                     gear_ratio=SwerveModuleFloatProperty(angle=150.0 / 7, drive=6.75),
                                     max_drive_speed=3,
                                     max_rotation_speed=math.pi / 6,
                                     fw_set_retries=5,
                                     fw_set_retry_delay_sec=0.05,
                                     invert_gyro=True,
                                     gyro_on_spi=True)
