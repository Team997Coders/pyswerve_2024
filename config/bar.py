from typing import NamedTuple
import rev
from config import MotorConfig, PIDConfig, AxisConfig


class BarConfig(NamedTuple):
    bar_motor: MotorConfig
    bar_pid: PIDConfig
    defualt_bar_angle: float
    bar_encoder_id: int
