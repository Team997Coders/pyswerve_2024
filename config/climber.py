from config import MotorConfig, PIDConfig
from dataclasses import dataclass

#climber_config = ClimberConfig(MotorConfig(id=14, inverted=False), climber_sensor_id=2, climber_sensor_inverted=True,
#                                climber_pid=PIDConfig(p=.2, i=0, d=0, wrapping=None), climber_max=1)
@dataclass
class ClimberConfig:
    climber1_motor: MotorConfig
    climber2_motor: MotorConfig
    climber_sensor_id: int
    climber_sensor_inverted: bool
    climber_pid: PIDConfig
    climber_max: int  # how many ticks of the encoder is the highest point of the climber
