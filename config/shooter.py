from typing import NamedTuple
from config import MotorConfig

class ShooterConfig(NamedTuple):
    left_motor: MotorConfig
    right_motor: MotorConfig
    left_flywheel_gear_ratio: float
    right_flywheel_gear_ratio: float
    left_flywheel_diameter_cm: float
    right_flywheel_diameter_cm: float
    default_velocity: float
    default_fire_time: float
    default_spinup_delay: float
