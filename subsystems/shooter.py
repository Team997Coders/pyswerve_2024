import commands2
import rev
from math_help import shared
from config import ShooterConfig
from config import PIDConfig
import hardware
import math


class Shooter(commands2.Subsystem):
    _left_motor: rev.CANSparkMax
    _right_motor: rev.CANSparkMax
    _left_encoder: rev.SparkRelativeEncoder
    _right_encoder: rev.SparkRelativeEncoder
    _pid: rev.SparkMaxPIDController

    def __init__(self, config: ShooterConfig, pid_config: PIDConfig):
        super().__init__()
        self._left_motor = rev.CANSparkMax(config.left_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self._right_motor = rev.CANSparkMax(config.right_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self._left_motor, config.left_motor)
        hardware.init_motor(self._right_motor, config.right_motor)

        self._right_motor.follow(self._left_motor, invert=config.right_motor.inverted)
        self._left_encoder = self._left_motor.getEncoder()
        self._right_encoder = self._right_motor.getEncoder()

        self._right_encoder.setPositionConversionFactor(
            1 / config.right_flywheel_gear_ratio * ((config.right_flywheel_diameter_cm / 100) * math.pi))
        self._left_encoder.setPositionConversionFactor(
            1 / config.left_flywheel_gear_ratio * (config.left_flywheel_diameter_cm / 100) * math.pi)

        self._right_encoder.setVelocityConversionFactor(
            (1 / config.right_flywheel_gear_ratio) * ((config.right_flywheel_diameter_cm / 100) * math.pi) / 60.0)
        self._left_encoder.setVelocityConversionFactor(
            (1 / config.left_flywheel_gear_ratio) * ((config.left_flywheel_diameter_cm / 100) * math.pi) / 60.0)

        self._pid = self._left_motor.getPIDController()
        hardware.init_pid(self._pid, pid_config, feedback_device=self._left_encoder)

    @property
    def velocity(self):
        return self._left_encoder.getVelocity()

    @velocity.setter
    def velocity(self, value):
        self._pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
