import rev
import wpilib
import subsystems
from robot_config import climber_constants


class Climber(subsystems):
    climberMotor: rev.CANSparkMax
    climberEncoder: rev.RelativeEncoder.EncoderType.kHallSensor

    def __init__(self):
        self.climberMotor = rev.CANSparkMax(climber_constants.climber_motor_id, rev.CANSparkMax.MotorType.kBrushless)
        self.climberEncoder = self.climberMotor.getEncoder()
        self.climberEncoder.setPosition(0)
        self.climberMotor.setIdleMode(self.climberMotor.getIdleMode().kBrake)

    def set_climber_motor_voltage(self, voltage: float):
        self.climberMotor.setVoltage(voltage)

    def get_climber_encoder_rotation(self):
        return self.climberEncoder.getPosition()

    def set_brake_mode(self):
        self.climberMotor.setIdleMode(self.climberMotor.getIdleMode().kBrake)
