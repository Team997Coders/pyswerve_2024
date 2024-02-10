import rev
import wpilib

import subsystems
from robot_config import indexer_constants

class Intake(subsystems):
    intakeNEO: rev.CANSparkMax
    intakeEncoder: rev.RelativeEncoder.EncoderType.kHallSensor
    def __init__(self):
        self.intakeNEO = rev.CANSparkMax(indexer_constants.intake_motor_id, rev.CANSparkMax.MotorType.kBrushless)
        self.intakeEncoder = self.intakeNEO.getEncoder()
        self.intake_pid = self.intakeNEO.getPIDController()
    @property
    def intake_velocity(self):
        return self.intakeNEO.getVelocity()
    @intake_velocity.setter
    def set_intake_velocity(self, value : float):
        self.feeder_pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
    @property
    def set_intake_voltage(self, voltage: float):
        self.intakeNEO.setVoltage(voltage)
    @property
    def get_intake_motor_voltage(self):
        return self.intakeNEO.getEncoder().getVelocity()
    @property
    def get_intake_motor_position(self):
        return self.intakeNEO.getEncoder().getPosition()

