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
    _climber_sensor: wpilib.DigitalInput | None
    _last_sensor_state: bool = False

    @property
    def pid(self) -> rev.SparkMaxPIDController:
        return self._pid

    def __init__(self, config: ClimberConfig, logger: logging.Logger):
        super().__init__()
        self.config = config
        self._logger = logger.getChild("Climber")
        self.climber_motor = rev.CANSparkMax(config.climber_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.climber_motor2 = rev.CANSparkMax(15, rev.CANSparkMax.MotorType.kBrushless)
        hardware.init_motor(self.climber_motor, config.climber_motor)
        self.climber_encoder = self.climber_motor.getEncoder()
        self.climber_encoder.setPosition(-1)
        self._pid = self.climber_motor.getPIDController()
        hardware.init_pid(self._pid, self.config.climber_pid, self.climber_encoder)
        self.set_brake_mode(True)

        self.climber_motor2.follow(self.climber_motor, True)
        try:
            self._climber_sensor = wpilib.DigitalInput(config.climber_sensor_id)
        except:
            print("Climber sensor not found")

    def set_climber_motor_voltage(self, voltage: float):
        self.climber_motor.setVoltage(voltage)

    def get_climber_encoder_rotation(self):
        return self.climber_encoder.getPosition()

    def set_brake_mode(self, brakeMode: bool):
        if brakeMode:
            self.climber_motor.setIdleMode(self.climber_motor.IdleMode.kBrake)
            self.climber_motor2.setIdleMode(self.climber_motor2.IdleMode.kBrake)
        else:
            self.climber_motor.setIdleMode(self.climber_motor.IdleMode.kCoast)
            self.climber_motor2.setIdleMode(self.climber_motor2.IdleMode.kCoast)

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
        #if self._climber_sensor:
        #    self.climber_motor.set(0)
        #else:
        self.climber_motor.set(value)
