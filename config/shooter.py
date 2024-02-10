from typing import NamedTuple

import wpilib
import rev

class ShooterConfig(NamedTuple):
    left_flywheel_id: int
    right_flywheel_id: int
    is_flywheel_inverted: bool
    default_flywheel_voltage: float
    relative_encoder_id: int