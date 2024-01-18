from .swerve_module_config import SwerveModuleFloatProperty, SwerveModuleIntProperty, OptionalSwerveModuleFloatProperty, OptionalSwerveModuleIntProperty

class PhysicalConfig:
    wheel_diameter_cm: float
    wheel_grip_coefficient_of_friction: float
    current_limit: OptionalSwerveModuleIntProperty
    ramp_rate: OptionalSwerveModuleFloatProperty
    encoder_pulses_per_revolution: SwerveModuleFloatProperty
    gear_ratio: SwerveModuleFloatProperty 

    def __init__(self,
                 wheel_diameter_cm: float,
                 wheel_grip_coefficient_of_friction: float,
                 
                 encoder_pulses_per_revolution: SwerveModuleFloatProperty, 
                 gear_ratio: SwerveModuleFloatProperty,
                 ramp_rate: OptionalSwerveModuleFloatProperty | None = None,
                 current_limit: OptionalSwerveModuleIntProperty | None = None): 
        current_limit = current_limit if current_limit is not None else OptionalSwerveModuleIntProperty()
        ramp_rate = ramp_rate if ramp_rate is not None else OptionalSwerveModuleFloatProperty()

        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheel_grip_coefficient_of_friction = wheel_grip_coefficient_of_friction
        self.current_limit = current_limit
        self.ramp_rate = ramp_rate
        self.encoder_pulses_per_revolution = encoder_pulses_per_revolution
        self.gear_ratio = gear_ratio 