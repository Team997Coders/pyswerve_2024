import subsystems
import rev
from robot_config import shooter_constants

class Shooter(subsystems):

    leftFlywheelNEO: rev.CANSparkMax
    rightFlywheelNEO: rev.CANSparkMax

    def __init__(self):
        self.leftFlywheelNEO = rev.CANSparkMax(shooter_constants.left_flywheel_id, rev.CANSparkMax.MotorType.kBrushless)
        self.rightFlywheelNEO = rev.CANSparkMax(shooter_constants.right_flywheel_id, rev.CANSparkMax.MotorType.kBrushless)
        self.rightFlywheelNEO.follow(self.leftFlywheelNEO)
        self.rightFlywheelNEO.setInverted(shooter_constants.is_flywheel_inverted)



    def set_left_motor_voltage(self, voltage : float):
       self.leftFlywheelNEO.setVoltage(voltage)



    def get_left_motor_voltage(self):
        return shooter_constants.defualt_flywheel_voltage
    def get_right_motor_voltage(self):  # delete me
        return shooter_constants.defualt_flywheel_voltage


    def get_left_flywheel_encoder_position(self):
        return self.leftFlywheelNEO.getEncoder().getPosition()


    def get_right_flywheel_encoder_position(self):
        return self.rightFlywheelNEO.getEncoder().getPosition()

    def get_left_flywheel_encoder_velocity(self):
        return self.leftFlywheelNEO.getEncoder().getVelocity()

    def get_right_flywheel_encoder_velocity(self):
        return self.rightFlywheelNEO.getEncoder().getVelocity()





