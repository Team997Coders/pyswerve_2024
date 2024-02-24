import commands2
import rev
from math_help import shared
from config import ShooterConfig
from config import PIDConfig
import hardware
import math
import logging


class Shooter(commands2.Subsystem):
    _left_motor: rev.CANSparkMax
    _right_motor: rev.CANSparkMax
    _left_encoder: rev.SparkRelativeEncoder
    _right_encoder: rev.SparkRelativeEncoder
    _pid: rev.SparkMaxPIDController
    _logger: logging.Logger
    config: ShooterConfig

    @property
    def pid(self) -> rev.SparkMaxPIDController:
        return self._pid

    def __init__(self, config: ShooterConfig, pid_config: PIDConfig, logger: logging.Logger):
        super().__init__()
        self.config = config
        self._logger = logger.getChild("Shooter")
        self._left_motor = rev.CANSparkMax(config.left_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self._right_motor = rev.CANSparkMax(config.right_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self._left_motor, config.left_motor)
        hardware.init_motor(self._right_motor, config.right_motor)

        self._right_motor.follow(self._left_motor, invert=config.right_motor.inverted)
        self._left_encoder = self._left_motor.getEncoder()
        self._right_encoder = self._right_motor.getEncoder()

        self._right_encoder.setPositionConversionFactor(
            (1 / config.right_flywheel_gear_ratio) * ((config.right_flywheel_diameter_cm / 100) * math.pi))
        self._left_encoder.setPositionConversionFactor(
            (1 / config.left_flywheel_gear_ratio) * (config.left_flywheel_diameter_cm / 100) * math.pi)

        self._right_encoder.setVelocityConversionFactor(
            (1 / config.right_flywheel_gear_ratio) * ((config.right_flywheel_diameter_cm / 100) * math.pi) / 60.0)
        self._left_encoder.setVelocityConversionFactor(
            (1 / config.left_flywheel_gear_ratio) * ((config.left_flywheel_diameter_cm / 100) * math.pi) / 60.0)

        self._pid = self._left_motor.getPIDController() 
        hardware.init_pid(self._pid, pid_config, feedback_device=self._left_encoder)

    @property
    def pid_config(self) -> PIDConfig:
        return PIDConfig(p=self._pid.getP(), i=self._pid.getI(), d=self._pid.getD())

    @property
    def velocity(self):
        return self._left_encoder.getVelocity()

    @velocity.setter
    def velocity(self, value: float):
        # print(f"Set shooter velocity {value}")
        self._pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)

    @property
    def speed(self):
        return self._left_motor.get()

    @speed.setter
    def speed(self, value: float):
        self._left_motor.set(value)

    def setVoltage(self, value: float):
        # print(f"Set shooter voltage {value}")
        self._left_motor.setVoltage(value)
        self._right_motor.setVoltage(value)
