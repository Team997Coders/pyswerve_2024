import wpilib
import rev
from config import *
from wpilib import SmartDashboard as sd



class SwerveModule():
    id: module_position

    drive_motor: rev.CANSparkMax
    angle_motor: rev.CANSparkMax

    drive_motor_encoder: rev.SparkRelativeEncoder
    angle_motor_encoder: rev.SparkRelativeEncoder

    drive_pid : rev.SparkMaxPIDController
    angle_pid : rev.SparkMaxPIDController

    absolute_encoder: rev.AbsoluteEncoder
 
    angle_pid_last_reference: float

    config: SwerveModuleConfig

    def __init__(self, id: module_position, module_config: SwerveModuleConfig, physical_config: PhysicalConfig):
        self.id = id 
        self.config = module_config
        self.angle_pid_last_reference = 0
        self.drive_motor = rev.CANSparkMax(module_config.drive_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self.angle_motor = rev.CANSparkMax(module_config.angle_motor.id, rev.CANSparkMax.MotorType.kBrushless)

        self.init_motor(self.drive_motor, module_config.drive_motor)
        self.init_motor(self.angle_motor, module_config.angle_motor)
        self.init_physical(physical_config)

        self.drive_pid = self.drive_motor.getPIDController()
        self.angle_pid = self.angle_motor.getPIDController()

        self.drive_motor_encoder = self.drive_motor.getEncoder()
        self.angle_motor_encoder = self.angle_motor.getEncoder()

        # Request specific angles from the PID controller 
        self.angle_pid.setP(0.05)
        
        self.angle_absolute_encoder = self.angle_motor.getAbsoluteEncoder(encoderType=rev.SparkAbsoluteEncoder.Type.kDutyCycle) 

        if module_config.encoder.conversion_factor is not None:
            self.angle_absolute_encoder.setPositionConversionFactor(module_config.encoder.conversion_factor)

        self.init_rotation(self.config)

    def report_to_dashboard(self):
        sd.putNumber(f"Drive Motor {self.id} Position", self.drive_motor_encoder.getPosition())
        sd.putNumber(f"Drive Motor {self.id} Velocity", self.drive_motor_encoder.getVelocity())
        sd.putNumber(f"Angle Motor {self.id} Position", self.angle_motor_encoder.getPosition())
        sd.putNumber(f"Angle Motor {self.id} Absolute", self.angle_absolute_encoder.getPosition())
        sd.putNumber(f"Angle Motor {self.id} Velocity", self.angle_motor_encoder.getVelocity())
        sd.putNumber(f"Angle PID {self.id} Reference", self.angle_pid_last_reference)

    def set_angle(self, angle: float):
        self.angle_pid.setReference(angle, rev.CANSparkMax.ControlType.kPosition)
        self.angle_pid_last_reference = angle

    @staticmethod
    def init_motor(motor: rev.CANSparkMax, config: MotorConfig):
        motor.restoreFactoryDefaults()
        motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        motor.setInverted(config.inverted)
    
    def init_physical(self, physical_config: PhysicalConfig):
        self.angle_motor.setSmartCurrentLimit(int(physical_config.current_limit.angle))
        self.drive_motor.setSmartCurrentLimit(int(physical_config.current_limit.drive))
    
    def init_rotation(self, config: SwerveModuleConfig):
        if config.encoder.offset is not None:
            self.angle_pid.setReference(config.encoder.offset, rev.CANSparkMax.ControlType.kPosition)
            self.angle_pid_last_reference = angle