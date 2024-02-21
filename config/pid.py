from dataclasses import dataclass
from typing import NamedTuple
from . import OptionalRange


class PositionVelocityConfig(NamedTuple):
    position: float
    velocity: float


class VelocityAccelerationConfig(NamedTuple):
    velocity: float
    acceleration: float


@dataclass
class PIDConfig:
    p: float
    i: float
    d: float
    wrapping: OptionalRange | None = None  # Wrapping values if PID is continuous
    tolerance: PositionVelocityConfig | None = None    # max velocity and acceleration

@dataclass
class ProfiledPIDConfig:
    p: float
    i: float
    d: float
    profile: VelocityAccelerationConfig
    wrapping: OptionalRange | None = None    # Wrapping values if PID is continuous
    tolerance: PositionVelocityConfig | None = None    # max velocity and acceleration


class FeedForwardConfig(NamedTuple):
    kS: float
    kV: float
    kA: float
