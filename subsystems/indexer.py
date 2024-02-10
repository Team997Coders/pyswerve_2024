import rev
import wpilib
import logging
import commands2
from typing import Callable, Any

import hardware
import logging
from robot_config import IndexerConfig


class Indexer(commands2.Subsystem):
    _indexer_motor: rev.CANSparkMax
    _indexer_encoder: rev.SparkRelativeEncoder
    _indexer_pid: rev.SparkMaxPIDController
    _config: IndexerConfig
    _feederSensor: wpilib.DigitalInput | None
    _logger: logging.Logger
    _index_sensor: wpilib.DigitalInput

    def __init__(self, config: IndexerConfig, logger: logging.Logger):
        super().__init__()
        self._logger = logger.getChild("Indexer")
        self._config = config
        try:
            self._feederSensor = wpilib.DigitalInput(config.indexer_sensor_id)
            self._read_indexer_state = lambda: not self._feederSensor.get() if config.indexer_sensor_inverted \
                else self._feederSensor
        except:
            self._logger.error("Could not initialize indexer sensor")
            self._feederSensor = None
            self._read_indexer_state = lambda: False

        self._indexer_motor = rev.CANSparkMax(config.motor_config.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self._indexer_motor, config.motor_config)
        self._indexer_encoder = self._indexer_motor.getEncoder()
        self._indexer_pid = self._indexer_motor.getPIDController()
        hardware.init_pid(self._indexer_pid, self._config.pid)

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
    def position(self) -> float:
        return self._indexer_encoder.getPosition()

    @position.setter
    def position(self, value):
        self._indexer_pid.setReference(value, rev.CANSparkMax.ControlType.kPosition)

    @property
    def velocity(self) -> float:
        return self._indexer_encoder.getVelocity()

    @velocity.setter
    def velocity(self, value):
        self._indexer_pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
