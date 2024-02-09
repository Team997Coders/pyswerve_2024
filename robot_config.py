import math_help
import math
from config import PIDConfig, DriverControlsConfig, MotorConfig, ModulePosition, SwerveModuleConfig, EncoderConfig, \
    OptionalRange, PhysicalConfig, OptionalSwerveModuleIntProperty, SwerveModuleFloatProperty, \
    OptionalSwerveModuleFloatProperty

default_angle_pid = PIDConfig(p=.6, i=0.0, d=0.2, wrapping=OptionalRange(min=0, max=math.pi * 2))
# Be Carefull when adding an i value to the drive pid, it can cause the robot to drive very fast
default_drive_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None)
default_rotation_pid = PIDConfig(p=.2, i=0.0, d=0.0, wrapping=OptionalRange(min=0, max=math.pi * 2))

joystick_controls = DriverControlsConfig(x_deadband=math_help.Range(0.15, 1),
                                         y_deadband=math_help.Range(0.15, 1),
                                         theta_deadband=math_help.Range(0.05, 1))
gamepad_controls = DriverControlsConfig(x_deadband=math_help.Range(0.10, 1),
                                        y_deadband=math_help.Range(0.10, 1),
                                        theta_deadband=math_help.Range(0.10, 1))

shooter_constants = ShooterConfig(left_flywheel_id = 3, right_flywheel_id = 4, is_flywheel_inverted=False, flywheel_voltage=2.0, relative_encoder_id=1)

indexer_constants = IndexerConfig(feeder_motor_id = 2, intake_motor_id = 3, feeder_sensor_channel = 0,is_feeder_motor_inverted = True, is_intake_motor_inverted = False)

climber_constants = ClimberConfig(climber_motor_id= 5, is_climber_motor_inverted= False)

swerve_modules = {ModulePosition.front_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=8, inverted=False),
                                         angle_motor=MotorConfig(id=1, inverted=True),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2),
                                         location=(9.5, 9.5),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.front_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=6, inverted=False),
                                         angle_motor=MotorConfig(id=7, inverted=True),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2),
                                         location=(9.5, -9.5),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.back_right:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=4, inverted=False),
                                         angle_motor=MotorConfig(id=5, inverted=True),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2),
                                         location=(-9.5, -9.5),
                                         angle_pid=default_angle_pid,
                                         drive_pid=default_drive_pid),
                  ModulePosition.back_left:
                      SwerveModuleConfig(drive_motor=MotorConfig(id=2, inverted=False),
                                         angle_motor=MotorConfig(id=3, inverted=True),
                                         encoder=EncoderConfig(id_val=None, offset=None, conversion_factor=math.pi * 2),
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
                                     invert_gyro=True)