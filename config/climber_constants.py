from typing import NamedTuple
import wpilib
import rev


class ClimberConfig(NamedTuple):
    climber_motor_id: int
    is_climber_motor_inverted: bool
