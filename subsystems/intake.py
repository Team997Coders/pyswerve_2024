import rev
import logging
import wpilib
import commands2

import hardware
from config import IntakeConfig


class Intake(commands2.Subsystem):
    intake_motor: rev.CANSparkMax
    intake_encoder: rev.SparkRelativeEncoder
    config: IntakeConfig
    _logger: logging.Logger


    def __init__(self, config: IntakeConfig, logger: logging.Logger):
        super().__init__()
        self.config = config
        self._logger = logger.getChild("Intake")
        self.intake_motor = rev.CANSparkMax(config.motor.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self.intake_motor, config.motor)
        self.intake_encoder = self.intake_motor.getEncoder()
        self.intake_encoder.setPositionConversionFactor(1)
        self.intake_encoder.setVelocityConversionFactor(1)

    @property
    def intake_velocity(self) -> float:
        return self.intake_encoder.getVelocity()

    @property
    def speed(self):
        return self.intake_motor.get()

    @speed.setter
    def speed(self, value: float):
        self.intake_motor.set(value)

    @property
    def voltage(self) -> float:
        return self.intake_motor.getAppliedOutput()

    @voltage.setter
    def voltage(self, value: float):
        self.intake_motor.setVoltage(value)
