import rev
import wpilib
import commands2
from typing import Callable
import hardware
import logging
from wpilib import SmartDashboard
from config import IndexerConfig


class Indexer(commands2.Subsystem):
    _indexer_motor: rev.CANSparkMax
    _indexer_encoder: rev.SparkRelativeEncoder
    config: IndexerConfig
    _indexer_sensor: wpilib.DigitalInput | None
    _logger: logging.Logger
    _read_indexer_state: Callable[[], bool]
    _last_sensor_state: bool = False
    _latched_sensor_state: bool = False

    @property
    def last_sensor_state(self) -> bool:
        return self._last_sensor_state

    def __init__(self, config: IndexerConfig, logger: logging.Logger):
        super().__init__()
        self._logger = logger.getChild("Indexer")
        self.config = config
        self._feederSensor = wpilib.DigitalInput(config.indexer_sensor_id)
        self._read_indexer_state = lambda: not self._feederSensor.get() if config.indexer_sensor_inverted \
            else lambda: self._feederSensor
        self._logger.info("Initialized indexer sensor")
        print("Initialized indexer sensor")

        self._indexer_motor = rev.CANSparkMax(config.motor_config.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self._indexer_motor, config.motor_config)
        self._indexer_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self._indexer_encoder = self._indexer_motor.getEncoder()
        self._indexer_encoder.setPositionConversionFactor(3 / 10)
        self._indexer_encoder.setVelocityConversionFactor(3 / 10)

    def get_sensor_status(self):
        if self._read_indexer_state:
            wpilib.Timer.start()
            SmartDashboard.putBoolean("Has Note", True)
            if wpilib.Timer.get() >3:
                SmartDashboard.putBoolean("Has Note", False)
        else:
            SmartDashboard.putBoolean("Has Note", False)

    def set_brake_mode(self):
        self._indexer_motor.setIdleMode(self._indexer_motor.IdleMode.kBrake)

    def clearNoteState(self) -> None:
        self._latched_sensor_state = False

    @property
    def ready(self) -> bool:
        return self._read_indexer_state()

    @property
    def voltage(self):
        return self._indexer_motor.getAppliedOutput()

    @property
    def speed(self) -> float:
        return self._indexer_motor.get()

    @speed.setter
    def speed(self, value: float):
        self._indexer_motor.set(value)

    @property
    def velocity(self) -> float:
        return self._indexer_encoder.getVelocity()
