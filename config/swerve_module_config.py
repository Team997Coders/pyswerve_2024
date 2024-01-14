import enum
import math

class module_position(enum.IntEnum):
    front_left = 0
    front_right = 1
    back_left = 2
    back_right = 3

class SwerveModuleFloatProperty():
    '''Stores a generic float value for drive & turn motors of a swerve module'''
    drive: float
    angle: float

    def __init__(self, drive: float, angle: float):
        self.drive = drive
        self.angle = angle

class SwerveModuleIntProperty():
    '''Stores a generic float value for drive & turn motors of a swerve module'''
    drive: int
    angle: int

    def __init__(self, drive: int, angle: int):
        self.drive = drive
        self.angle = angle

class MotorConfig():
    '''Information to configure a motor on the RoboRIO'''
    id: int #Motor ID on the RoboRIO
    inverted: bool # Invert the motor'

    def __init__(self, id: int, inverted: bool):
        self.id = id
        self.inverted = inverted

class EncoderConfig():
    '''Information to configure an encoder on the RoboRIO'''
    id: int | None # Encoder ID on the RoboRIO, if there is one
    offset: float | None # Offset in radians.  Subtract this number from the absolute encoder value to get 0 degrees relative to robot chassis
    conversion_factor: float | None # Conversion factor from encoder ticks to radians

    def __init__(self, id: int | None = None, offset: float | None = None, conversion_factor: float | None = None):
        self.id = id
        self.offset = offset
        self.conversion_factor = conversion_factor

        if self.offset is not None:
            while self.offset <= 0: # The absolute encoder cannot be negative, so add 2*pi until it is positive
                self.offset = self.offset + (math.pi * 2.0) 

class SwerveModuleConfig():
    '''Information to configure a swerve module'''
    drive_motor: MotorConfig
    angle_motor: MotorConfig
    encoder: EncoderConfig
    location: tuple[float, float] # (x, y) in meters

    def __init__(self, drive_motor: MotorConfig, turn_motor: MotorConfig, encoder: EncoderConfig, location: tuple[float, float]):
        self.drive_motor = drive_motor
        self.angle_motor = turn_motor
        self.encoder = encoder
        self.location = location

