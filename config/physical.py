from .swerve_module import SwerveModuleFloatProperty, SwerveModuleIntProperty, OptionalSwerveModuleFloatProperty, \
    OptionalSwerveModuleIntProperty
import math

class PhysicalConfig:
    wheel_diameter_cm: float # Diameter of the wheel in centimeters
    wheel_grip_coefficient_of_friction: float # Coefficient of friction between the wheel and the ground
    encoder_pulses_per_revolution: SwerveModuleFloatProperty # Pulses per revolution of the encoder
    gear_ratio: SwerveModuleFloatProperty # Ratio of motor rotations to wheel rotations
    max_drive_speed: float # Meters per second
    max_rotation_speed: float # Radians per second   invert_gyro: bool # Whether the gyro is inverted
    gyro_on_spi: bool # If false, the gyro is on i2c
    def __init__(self,
                 wheel_diameter_cm: float,
                 wheel_grip_coefficient_of_friction: float,
                 encoder_pulses_per_revolution: SwerveModuleFloatProperty, 
                 gear_ratio: SwerveModuleFloatProperty,
                 max_drive_speed: float = 1.0, 
                 max_rotation_speed: float = math.pi / 4, 

                 invert_gyro: bool = False,
                 gyro_on_spi: bool = True):
        self.wheel_diameter_cm = wheel_diameter_cm
        self.wheel_grip_coefficient_of_friction = wheel_grip_coefficient_of_friction
        self.encoder_pulses_per_revolution = encoder_pulses_per_revolution
        self.gear_ratio = gear_ratio 
        self.max_drive_speed = max_drive_speed
        self.max_rotation_speed = max_rotation_speed
        self.invert_gyro = invert_gyro
        self.gyro_on_spi = gyro_on_spi

        if self.wheel_diameter_cm <= 0:
            raise ValueError("Wheels must have a positive diameter")

        if self.wheel_grip_coefficient_of_friction <= 0:
            raise ValueError("In this universe the coefficient of friction must be greater than 0")