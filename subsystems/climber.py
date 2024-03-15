import rev
import wpilib
import subsystems
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
        self.climber_motor = rev.CANSparkMax(config.climber1_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.climber_motor2 = rev.CANSparkMax(config.climber2_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self.climber_motor, config.climber1_motor)
        hardware.init_motor(self.climber_motor2, config.climber2_motor)
        self.climber_encoder = self.climber_motor.getEncoder()
        self.climber_encoder.setPosition(-1)
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)
        self.climber_motor2.setIdleMode(self.climber_motor2.getIdleMode().kBrake)
        self._pid = self.climber_motor.getPIDController()
        self.climber_motor2.follow(self.climber_motor)
        hardware.init_pid(self._pid, self.config.climber_pid, self.climber_encoder)
        self._climberSensor = wpilib.DigitalInput(config.climber_sensor_id)
        self._read_climber_state = lambda: not self._climberSensor.get() if config.climber_sensor_inverted \
            else lambda: self._climberSensor
        self._logger.info("Initialized climber sensor")
        print("Initialized climber sensor")

    @property
    def ready(self) -> bool:
        return self._read_climber_state()

    def get_climber_position(self):
        return self.climber_encoder.getPosition()

    def set_brake_mode(self):
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)
        self.climber2_motor.setIdleMode(self.climber2_motor.getIdleMode().kBrake)

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
