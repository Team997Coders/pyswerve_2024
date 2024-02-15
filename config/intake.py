from typing import NamedTuple

import wpilib
import rev
from config import MotorConfig, PIDConfig

class IntakeConfig(NamedTuple):
    motor: MotorConfig
    pid: PIDConfig
    defualt_velocity: float
