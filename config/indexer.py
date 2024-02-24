from typing import NamedTuple
from config import MotorConfig, PIDConfig


class IndexerConfig(NamedTuple):
    motor_config: MotorConfig
    indexer_sensor_id: int
    indexer_sensor_inverted: bool
    pid: PIDConfig
    default_intake_velocity: float
    default_shoot_velocity: float
