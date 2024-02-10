import subsystems
import rev
from robot_config import shooter_constants

class Shooter(subsystems):

    left_flywheel_NEO: rev.CANSparkMax
    right_flywheel_NEO: rev.CANSparkMax
    # right_pid : rev.CANSparkMax.getPIDController()
    # left_pid : rev.CANSparkMax.getPIDController()


    def __init__(self):
        self.left_flywheel_NEO = rev.CANSparkMax(shooter_constants.left_flywheel_id, rev.CANSparkMax.MotorType.kBrushless)
        self.right_flywheel_NEO = rev.CANSparkMax(shooter_constants.right_flywheel_id, rev.CANSparkMax.MotorType.kBrushless)
        self.right_flywheel_NEO.follow(self.left_flywheel_NEO)
        self.right_flywheel_NEO.setInverted(shooter_constants.is_flywheel_inverted)
        self.right_encoder = self.right_flywheel_NEO.getEncoder()
        self.left_encoder = self.left_flywheel_NEO.getEncoder()

        self.right_pid = self.right_flywheel_NEO.getPIDController()
        self.left_pid = self.left_flywheel_NEO.getPIDController()
        self.right_encoder.setPositionConversionFactor(1 / shooter_constants.right_flywheel_gear_ratio * ((shooter_constants.right_flywheel_diameter_cm / 100) * math.pi))
        self.left_encoder.setPositionConversionFactor(1 / shooter_constants.left_flywheel_gear_ratio * (shooter_constants.left_flywheel_diameter_cm / 100) * math.pi)

        self.right_encoder.setVelocityConversionFactor((1 / shooter_constants.right_flywheel_gear_ratio) * ((shooter_constants.right_flywheel_diameter_cm / 100) * math.pi) / 60.0)
        self.left_encoder.setVelocityConversionFactor((1 / shooter_constants.left_flywheel_gear_ratio) * ((shooter_constants.left_flywheel_diameter_cm / 100) * math.pi) / 60.0)
        init_pid(self.right_pid, robot_config.default_right_flywheel_pid, feedback_device=self.right_encoder)
        init_pid(self.left_pid, robot_config.default_left_flywheel_pid, feedback_device=self.left_encoder)


    def get_left_motor_voltage(self):
        return shooter_constants.defualt_flywheel_voltage

    @property
    def left_flywheel_encoder_position(self):
        return self.left_flywheel_NEO.getEncoder().getPosition()

    @property
    def left_flywheel_velocity(self):
        return self.left_flywheel_NEO.getEncoder().getVelocity()

    @left_flywheel_velocity.setter
    def left_flywheel_velocity(self, value : float):
        self.left_pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)

        # self._drive_motor = rev.CANSparkMax(module_config.drive_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        # self._angle_motor = rev.CANSparkMax(module_config.angle_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        #
        # self.init_motor(self._drive_motor, module_config.drive_motor)
        # self.init_motor(self._angle_motor, module_config.angle_motor)
        # self.init_physical(physical_config)
        #
        # self.drive_pid = self._drive_motor.getPIDController()
        # self.angle_pid = self._angle_motor.getPIDController()
        #
        # self.drive_motor_encoder = self._drive_motor.getEncoder()
        # self.angle_motor_encoder = self._angle_motor.getEncoder()
        #
        # self.drive_motor_encoder.setPositionConversionFactor(1.0 / (physical_config.gear_ratio.drive * ((physical_config.wheel_diameter_cm / 100) * math.pi)))
        # # Use the line below to report number of rotations for wheel rotation tests
        # # self.drive_motor_encoder.setPositionConversionFactor(1.0 / physical_config.gear_ratio.drive)
        # self.drive_motor_encoder.setVelocityConversionFactor((1.0 / physical_config.gear_ratio.drive) * (
        #         (physical_config.wheel_diameter_cm / 100) * math.pi) / 60.0)  # convert from rpm to revolutions/sec
        #
        # # Request specific angles from the PID controller
        # self.init_pid(self.drive_pid, module_config.drive_pid, feedback_device=self.drive_motor_encoder)


