import enum

from . import MotorConfig, EncoderConfig
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


class SwerveModuleConfig(NamedTuple):
    """Information to configure a swerve module"""
    drive_motor: MotorConfig
    angle_motor: MotorConfig
    encoder: EncoderConfig
    location: tuple[float, float]  # (x, y) in meters
    drive_pid: PIDConfig
    angle_pid: PIDConfig