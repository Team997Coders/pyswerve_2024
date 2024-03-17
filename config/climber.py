from typing import NamedTuple
import wpilib
import rev
from config import MotorConfig, PIDConfig, AxisConfig


class ClimberConfig(NamedTuple):
    climber_motor: MotorConfig
    climber2_motor: MotorConfig
    climber_sensor_id: int
    climber_sensor_inverted: bool
    climber_pid: PIDConfig
    climber_max: int  # how many ticks of the encoder is the highest point of the climber
