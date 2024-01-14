from .swerve_module_config import SwerveModuleFloatProperty, SwerveModuleIntProperty

class PhysicalConfig:
    wheel_diameter_cm: float
    wheel_grip_coefficient_of_friction: float
    current_limit: SwerveModuleIntProperty
    ramp_rate: SwerveModuleFloatProperty
    encoder_pulses_per_revolution: SwerveModuleFloatProperty
    gear_ratio: SwerveModuleFloatProperty 

    def __init__(self,
                 wheel_diameter_cm: float,
                 wheel_grip_coefficient_of_friction: float,
                 current_limit: SwerveModuleIntProperty,
                 ramp_rate: SwerveModuleFloatProperty,
                 encoder_pulses_per_revolution: SwerveModuleFloatProperty, 
                 gear_ratio: SwerveModuleFloatProperty):
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheel_grip_coefficient_of_friction = wheel_grip_coefficient_of_friction
        self.current_limit = current_limit
        self.ramp_rate = ramp_rate
        self.encoder_pulses_per_revolution = encoder_pulses_per_revolution
        self.gear_ratio = gear_ratio 