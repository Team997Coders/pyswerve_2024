import rev
from wpilib import DigitalInput
import logging
import commands2
import config
from config import ClimberConfig

import hardware


class Climber(commands2.Subsystem):
    climber_motor: rev.CANSparkMax
    climber_encoder: rev.RelativeEncoder.EncoderType.kHallSensor
    config: config.ClimberConfig
    _logger: logging.Logger

    @property
    def pid(self) -> rev.SparkMaxPIDController:
        return self._pid

    def __init__(self, config: ClimberConfig, logger: logging.Logger):
        super().__init__()
        self.config = config
        self._logger = logger.getChild("Climber")
        self.climber_motor = rev.CANSparkMax(self.config.climber_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.climber2_motor = rev.CANSparkMax(self.config.climber2_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.climber2_motor.follow(self.climber_motor)
        self.climber_sensor = DigitalInput(config.climber_sensor_id)
        self.read_climber_state = lambda: not self.climber_sensor.get() if config.climber_sensor_inverted else lambda: self.climber_sensor
        hardware.init_motor(self.climber_motor, config.climber_motor)
        self.climber_encoder = self.climber_motor.getEncoder()
        self.climber_encoder.setPosition(-1)
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)
        self.climber2_motor.setIdleMode(self.climber2_motor.getIdleMode().kBrake)

        self._pid = self.climber_motor.getPIDController()
        hardware.init_pid(self._pid, self.config.climber_pid, self.climber_encoder)

    def get_climber_sensor_status(self):
        self.climber_sensor.get()

    def set_climber_motor_voltage(self, voltage: float):
        self.climber_motor.setVoltage(voltage)

    def get_climber_encoder_rotation(self):
        return self.climber_encoder.getPosition()

    def set_brake_mode(self):
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)
        self.climber2_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)

    def set_coast_mode(self):
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kCoast)
        self.climber2_motor.setIdleMode(self.climber_motor.getIdleMode().kCoast)
    @property
    def position(self):
        return self.climber_encoder.getPosition()

    @position.setter
    def position(self, value):
        self._pid.setReference(value, rev.CANSparkMax.ControlType.kPosition)

    @property
    def speed(self) -> float:
        return self.climber_motor.get()

    @speed.setter
    def speed(self, value: float):
        self.climber_motor.set(value)
