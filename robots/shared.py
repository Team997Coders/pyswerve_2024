import math_help
from config import AxisConfig, DriverControlsConfig, MotorConfig, ModulePosition, SwerveModuleConfig, EncoderConfig, \
    OptionalRange, PhysicalConfig, OptionalSwerveModuleIntProperty, SwerveModuleFloatProperty, \
    OptionalSwerveModuleFloatProperty, ShooterConfig, IndexerConfig, IntakeConfig, ClimberConfig, \
    ProfiledPIDConfig, VelocityAccelerationConfig, PositionVelocityConfig, FeedForwardConfig, SwerveModuleIntProperty

swerve_current_limit = SwerveModuleIntProperty(drive=40, angle=40)
swerve_ramp_rate = OptionalSwerveModuleFloatProperty(drive=0.25, angle=0.25)
