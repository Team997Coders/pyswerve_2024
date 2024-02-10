import rev
import wpilib
import subsystems
import commands2
import config
from config import ClimberConfig
import hardware


class Climber(commands2.Subsystem):
    climber_motor: rev.CANSparkMax
    climber_encoder: rev.RelativeEncoder.EncoderType.kHallSensor
    _config: config.ClimberConfig

    def __init__(self, config: ClimberConfig):
        super().__init__()
        self._config = config
        self.climber_motor = rev.CANSparkMax(self._config.climber_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.climber_encoder = self.climber_motor.getEncoder()
        self.climber_encoder.setPosition(0)
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)
        self._pid = self.climber_motor.getPIDController()
        hardware.init_pid(self._pid, self._config.climber_pid, self.climber_encoder)

    def set_climber_motor_voltage(self, voltage: float):
        self.climber_motor.setVoltage(voltage)

    def get_climber_encoder_rotation(self):
        return self.climber_encoder.getPosition()

    def set_brake_mode(self):
        self.climber_motor.setIdleMode(self.climber_motor.getIdleMode().kBrake)

    @property
    def velocity(self) -> float:
        return self.climber_encoder.getVelocity()

    @velocity.setter
    def velocity(self, value):
        self._pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
