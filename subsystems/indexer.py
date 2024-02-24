import rev
import wpilib
import commands2
from typing import Callable

import hardware
import logging
from config import IndexerConfig


class Indexer(commands2.Subsystem):
    _indexer_motor: rev.CANSparkMax
    _indexer_encoder: rev.SparkRelativeEncoder
    _indexer_pid: rev.SparkMaxPIDController
    config: IndexerConfig
    _indexer_sensor: wpilib.DigitalInput | None
    _logger: logging.Logger
    _read_indexer_state: Callable[[], bool]

    @property
    def pid(self) -> rev.SparkMaxPIDController:
        return self._indexer_pid

    def __init__(self, config: IndexerConfig, logger: logging.Logger):
        super().__init__()
        self._logger = logger.getChild("Indexer")
        self.config = config
        try:
            self._feederSensor = wpilib.DigitalInput(config.indexer_sensor_id)
            self._read_indexer_state = lambda: not self._feederSensor.get() if config.indexer_sensor_inverted \
                else self._feederSensor
            self._logger.info("Initialized indexer sensor")
            print("Initialized indexer sensor")
        except:
            self._logger.error("Could not initialize indexer sensor")
            print("Could not initialize indexer sensor")
            self._feederSensor = None
            self._read_indexer_state = lambda: False

        self._indexer_motor = rev.CANSparkMax(config.motor_config.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self._indexer_motor, config.motor_config)
        self._indexer_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self._indexer_encoder = self._indexer_motor.getEncoder()
        self._indexer_encoder.setPositionConversionFactor(3 / 10)
        self._indexer_pid = self._indexer_motor.getPIDController()
        hardware.init_pid(self._indexer_pid, self.config.pid)

    @property
    def ready(self) -> bool:
        return self._read_indexer_state()

    @property
    def voltage(self):
        return self._indexer_motor.getBusVoltage()

    @voltage.setter
    def voltage(self, value: float):
        self._indexer_motor.setVoltage(value)

    @property
    def velocity(self) -> float:
        return self._indexer_encoder.getVelocity()

    @velocity.setter
    def velocity(self, value):
        # if value == 0:
        # self._indexer_encoder.setPosition(0)
        # self._indexer_pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
        self._indexer_motor.set(1)
