from config import MotorConfig, PIDConfig
from dataclasses import dataclass


@dataclass
class IntakeConfig:
    motor: MotorConfig
    pid: PIDConfig
    intake_velocity: float  # Speed for pulling the note off the ground
    outtake_velocity: float # How fast to run the motor when ejecting a note without shooting it
