from config import MotorConfig, PIDConfig
from dataclasses import dataclass


@dataclass
class IndexerConfig:
    motor_config: MotorConfig
    indexer_sensor_id: int
    indexer_sensor_inverted: bool
    pid: PIDConfig
    intake_velocity: float #We tend to load slower to ensure breakbeam is triggered
    shoot_velocity:  float #We tend to shoot faster to ensure we get the power we want
    outtake_velocity: float #How fast to run the motor when ejecting a note without shooting it

