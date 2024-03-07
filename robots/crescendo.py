import math_help
import math
from config import PIDConfig, DriverControlsConfig, MotorConfig, ModulePosition, SwerveModuleConfig, EncoderConfig, \
    OptionalRange, PhysicalConfig, OptionalSwerveModuleIntProperty, SwerveModuleFloatProperty, \
    OptionalSwerveModuleFloatProperty, ShooterConfig, IndexerConfig, IntakeConfig, ClimberConfig, \
    ProfiledPIDConfig, VelocityAccelerationConfig, PositionVelocityConfig, FeedForwardConfig, SwerveModuleIntProperty, \
    AxisConfig, PhotonCameraConfig, LimelightCameraConfig
import wpimath.geometry as geom
from .common import swerve_current_limit, swerve_ramp_rate

# Panel is #13

#Set to true if the robot has mechanisms beyond navigation, vision, and swerve
has_mechanisms = True


default_angle_pid = PIDConfig(p=.6, i=0.0, d=0.2, wrapping=OptionalRange(min=0, max=math.pi * 2), tolerance=None)
# Be Careful when adding an i value to the drive pid, it can cause the robot to drive very fast
default_drive_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None, tolerance=None)
default_heading_pid = ProfiledPIDConfig(p=.25, i=0.1, d=0.001,
                                        wrapping=OptionalRange(min=-math.pi, max=math.pi),
                                        profile=VelocityAccelerationConfig(velocity=math.pi * 5,
                                                                           acceleration=4 * math.pi),
                                        tolerance=PositionVelocityConfig(position=math.pi / 360, velocity=0.01)
                                        )
default_axis_pid = ProfiledPIDConfig(p=6, i=0.12, d=0.001,
                                     profile=VelocityAccelerationConfig(velocity=15, acceleration=5),
                                     tolerance=PositionVelocityConfig(position=0.05, velocity=0.05))
default_heading_feedforward = FeedForwardConfig(kS=0.0,
                                                kV=0.01,
                                                kA=0.001)
default_flywheel_pid = PIDConfig(p=0.2, i=0.0, d=0.05, wrapping=None, tolerance=None)

joystick_controls = DriverControlsConfig(x_deadband=math_help.Range(0.2, 1),
                                         y_deadband=math_help.Range(0.2, 1),
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
                               default_velocity=1,
                               default_fire_time=.5,
                               default_spinup_delay=1)  # add motor configs
indexer_config = IndexerConfig(MotorConfig(id=10, inverted=False), indexer_sensor_id=0, indexer_sensor_inverted=True,
                               pid=PIDConfig(p=.000001, i=0, d=0, wrapping=None, tolerance=None),
                               intake_velocity=.4, shoot_velocity=1, outtake_velocity=-1)  # fix feeder_sensor_id
intake_config = IntakeConfig(MotorConfig(id=15, inverted=True), pid=PIDConfig(p=.000001, i=0, d=0, wrapping=None),
                             intake_velocity=.5, outtake_velocity=-1)
climber_config = ClimberConfig(MotorConfig(id=14, inverted=False), climber_pid=PIDConfig(p=.2, i=0, d=0, wrapping=None),
                               climber_max=1)

photon_camera_config = PhotonCameraConfig(camera_position=geom.Transform3d(geom.Translation3d(0, 0, 0), #camera postition on the robot xyz in meters from the center, CURRENTLY UNMEASURED
                                                              geom.Rotation3d(0, 0, 0)), #camera rotation on the robot in degrees, CURRENTLY UNMEASURED
                                          camera_name="set name here" #Set name from local host window
                                          )

limelight_camera_config = LimelightCameraConfig(camera_position=geom.Transform3d(geom.Translation3d(0, 0, 0), #camera postition on the robot xyz in meters from the center, CURRENTLY UNMEASURED
                                                                geom.Rotation3d(0, 0, 0)), #camera rotation on the robot in degrees, CURRENTLY UNMEASURED
                                 camera_name=None, #Set name from nettable name if not default of 'limelight'
                                 refresh_rate=5
                                 )

physical_properties = PhysicalConfig(wheel_diameter_cm=10.16,
                                     wheel_grip_coefficient_of_friction=1,
                                     encoder_pulses_per_revolution=SwerveModuleFloatProperty(drive=1, angle=1),
                                     gear_ratio=SwerveModuleFloatProperty(angle=150.0 / 7, drive=6.75),
                                     max_drive_speed=5,
                                     max_rotation_speed=math.pi / 6,
                                     invert_gyro=False,
                                     gyro_on_spi=True,
                                     )

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

standard_joystick_drive_axis_config = AxisConfig(deadband=math_help.Range(0.15, 1),
                                                 output_range=math_help.Range(0,
                                                                              physical_properties.max_drive_speed))

standard_joystick_rotation_axis_config = AxisConfig(deadband=math_help.Range(0.5, 1),
                                                    output_range=math_help.Range(0,
                                                                                 physical_properties.max_drive_speed))

standard_gamepad_drive_axis_config = AxisConfig(deadband=math_help.Range(0.10, 1),
                                                output_range=math_help.Range(0,
                                                                             physical_properties.max_drive_speed))

standard_joystick_climber_axis_config = AxisConfig(deadband=math_help.Range(0.15, 1),
                                                   output_range=math_help.Range(-1,
                                                                                climber_config.climber_max))
