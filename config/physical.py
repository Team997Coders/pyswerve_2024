from .swerve_module import SwerveModuleFloatProperty, SwerveModuleIntProperty, OptionalSwerveModuleFloatProperty, \
    OptionalSwerveModuleIntProperty
import math

class PhysicalConfig:
    wheel_diameter_cm: float # Diameter of the wheel in centimeters
    wheel_grip_coefficient_of_friction: float # Coefficient of friction between the wheel and the ground
    current_limit: OptionalSwerveModuleIntProperty # Current limit in amps
    ramp_rate: OptionalSwerveModuleFloatProperty 
    encoder_pulses_per_revolution: SwerveModuleFloatProperty # Pulses per revolution of the encoder
    gear_ratio: SwerveModuleFloatProperty # Ratio of motor rotations to wheel rotations
    max_drive_speed: float # Meters per second
    max_rotation_speed: float # Radians per second
    fw_set_retry_delay_sec: float # How long to wait between retries if hardware returns an error during configuration
    fw_set_retries: int # How many times to retry if hardware returns an error during configuration
    invert_gyro: bool # Whether the gyro is inverted
    gyro_on_spi: bool # If false, the gyro is on i2c
    def __init__(self,
                 wheel_diameter_cm: float,
                 wheel_grip_coefficient_of_friction: float,
                 
                 encoder_pulses_per_revolution: SwerveModuleFloatProperty, 
                 gear_ratio: SwerveModuleFloatProperty,
                 ramp_rate: OptionalSwerveModuleFloatProperty | None = None,
                 current_limit: OptionalSwerveModuleIntProperty | None = None,
                 max_drive_speed: float = 1.0, 
                 max_rotation_speed: float = math.pi / 4, 
                 fw_set_retry_delay_sec : float = 0.01,
                 fw_set_retries: int = 5,
                 invert_gyro: bool = False,
                 gyro_on_spi: bool = True):
        current_limit = current_limit if current_limit is not None else OptionalSwerveModuleIntProperty(None, None)
        ramp_rate = ramp_rate if ramp_rate is not None else OptionalSwerveModuleFloatProperty(None, None)

        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheel_grip_coefficient_of_friction = wheel_grip_coefficient_of_friction
        self.current_limit = current_limit
        self.ramp_rate = ramp_rate
        self.encoder_pulses_per_revolution = encoder_pulses_per_revolution
        self.gear_ratio = gear_ratio 
        self.max_drive_speed = max_drive_speed
        self.max_rotation_speed = max_rotation_speed
        self.fw_set_retries = fw_set_retries
        self.fw_set_retry_delay_sec = fw_set_retry_delay_sec
        self.invert_gyro = invert_gyro
        self.gyro_on_spi = gyro_on_spi

        if self.wheel_diameter_cm <= 0:
            raise ValueError("Wheels must have a positive diameter")

        if self.wheel_grip_coefficient_of_friction <= 0:
            raise ValueError("In this universe the coefficient of friction must be greater than 0")
 
        if self.fw_set_retries < 0:
            raise ValueError("Number of retries must be greater than or equal to 0")
        
        if  not 0 < self.fw_set_retry_delay_sec <= 1 :
            raise ValueError("Retry delay must be greater than 0 and less than 1 second")