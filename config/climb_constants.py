from typing import NamedTuple
import wpilib
import rev
from config import MotorConfig, PIDConfig, AxisConfig


class ClimberConfig(NamedTuple):
    climber_motor: MotorConfig
    climber_pid: PIDConfig
