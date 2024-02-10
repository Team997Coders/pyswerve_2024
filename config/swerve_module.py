import enum
import math
from .pid import PIDConfig
from typing import NamedTuple


class ModulePosition(enum.IntEnum):
    front_left = 0
    front_right = 1
    back_right = 2
    back_left = 3

    def __str__(self):
        return enum.Enum.__str__(self).replace("module_position.", "").replace("_", " ")


class OptionalSwerveModuleFloatProperty(NamedTuple):
    """Stores an optional generic float value for drive & turn motors of a swerve module.  Use None to leave value unconfigured"""
    drive: float | None
    angle: float | None


class SwerveModuleFloatProperty(NamedTuple):
    """Stores a required float value for drive & turn motors of a swerve module"""
    drive: float
    angle: float


class OptionalSwerveModuleIntProperty(NamedTuple):
    """Stores an optional generic int value for drive & turn motors of a swerve module.  Use None to leave value unconfigured"""
    drive: int | None
    angle: int | None


class SwerveModuleIntProperty(NamedTuple):
    """Stores a required float value for drive & turn motors of a swerve module"""
    drive: int
    angle: int


class MotorConfig(NamedTuple):
    """Information to configure a motor on the RoboRIO"""
    id: int  # Motor ID on the RoboRIO
    inverted: bool  # Invert the motor


class EncoderConfig:
    """Information to configure an encoder on the RoboRIO"""
    id: int | None  # Encoder ID on the RoboRIO, if there is one
    offset: float | None  # Offset in radians.  Subtract this number from the absolute encoder value to get 0 degrees relative to robot chassis. Set to None if offset is configured in Rev Hardware Client
    conversion_factor: float | None  # Conversion factor from encoder ticks to radians

    def __init__(self, id_val: int | None = None, offset: float | None = None, conversion_factor: float | None = None):
        self.id = id_val
        self.offset = offset
        self.conversion_factor = conversion_factor

        if self.offset is not None:
            while self.offset <= 0:  # The absolute encoder cannot be negative, so add 2*pi until it is positive
                self.offset = self.offset + (math.pi * 2.0)


class SwerveModule(NamedTuple):
    """Information to configure a swerve module"""
    drive_motor: MotorConfig
    angle_motor: MotorConfig
    encoder: EncoderConfig
    location: tuple[float, float]  # (x, y) in meters
    drive_pid: PIDConfig
    angle_pid: PIDConfig