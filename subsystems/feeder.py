import rev
import wpilib

import subsystems
from robot_config import indexer_constants

class Indexer(subsystems):
    intakeNEO: rev.CANSparkMax
    feederNEO: rev.CANSparkMax
    feederEncoder: rev.RelativeEncoder.EncoderType.kHallSensor
    intakeEncoder: rev.RelativeEncoder.EncoderType.kHallSensor
    def __init__(self):
        self.feederNEO = rev.CANSparkMax(indexer_constants.feeder_motor_id, rev.CANSparkMax.MotorType.kBrushless)
        self.feederEncoder = self.feederNEO.getEncoder()
        self.feederSensor = wpilib.DigitalInput(indexer_constants.feeder_sensor_channel)
        self.feeder_pid = self.feederNEO.getPIDController()
    @property
    def get_sensor_status(self):
        return not self.feederSensor.get()
    @property
    def set_feeder_voltage(self, voltage: float):
        self.setVoltage(voltage)

    def set_intake_voltage(self, voltage: float):
        self.intakeNEO.setVoltage(voltage)

    def get_intake_motor_voltage(self):
        return self.intakeNEO.getEncoder().getVelocity()

    def get_feeder_motor_voltage(self):
        return self.feederNEO.getEncoder().getVelocity()

    def get_intake_motor_position(self):
        return self.intakeNEO.getEncoder().getPosition()

    def get_feeder_motor_position(self):
        return self.feederNEO.getEncoder().getPosition()

