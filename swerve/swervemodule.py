import wpilib
import rev
import math
from config import *
from wpilib import SmartDashboard as sd
from math_help import shortest_angle_difference


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

    rel_to_absolute_angle_adjustment: float # How far we need to adjust the pid reference to get the absolute encoder to read the correct angle
    rel_to_corrected_angle_adjustment: float # How far we need to adjust the pid reference to get the angle relative to robot chassis
    config: SwerveModuleConfig

    def __init__(self, id: module_position, module_config: SwerveModuleConfig, physical_config: PhysicalConfig):
        self.id = id 
        self.rel_to_absolute_angle_adjustment = 0
        self.rel_to_corrected_angle_adjustment = 0
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
        self.angle_pid.setP(0.4)
        self.angle_pid.setI(0)
        self.angle_pid.setD(0.01)
        
        self.angle_motor_encoder.setPositionConversionFactor((1.0 / physical_config.gear_ratio.angle) * math.pi * 2.0)    
        self.angle_absolute_encoder = self.angle_motor.getAbsoluteEncoder(encoderType=rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        
        if module_config.encoder.conversion_factor is not None:
            self.angle_absolute_encoder.setPositionConversionFactor(module_config.encoder.conversion_factor)


        self.angle_pid.setFeedbackDevice(self.angle_absolute_encoder)

        self.init_rotation()

    def report_to_dashboard(self):
        sd.putNumber(f"Drive {self.id} Position", self.drive_motor_encoder.getPosition())
        sd.putNumber(f"Drive {self.id} Velocity", self.drive_motor_encoder.getVelocity())
        sd.putNumber(f"Angle {self.id} Position", self.angle_motor_encoder.getPosition())# * 180 / math.pi)
        sd.putNumber(f"Angle {self.id} Absolute", self.angle_absolute_encoder.getPosition())# * 180 / math.pi)
        sd.putNumber(f"Angle {self.id} Velocity", self.angle_motor_encoder.getVelocity())
        sd.putNumber(f"Angle PID {self.id} Reference", self.angle_pid_last_reference)
        sd.putNumber(f"Rel to Abs {self.id} adjust", self.rel_to_absolute_angle_adjustment)
        sd.putNumber(f"Rel to Chassis {self.id} adjust", self.rel_to_corrected_angle_adjustment)
        

    def set_angle(self, angle: float):
        '''Set the angle of the wheel in radians, travel the shortest distance to requested angle'''
        pid_angle = angle + self.rel_to_corrected_angle_adjustment
        self.angle_pid.setReference(self.clamp_angle(pid_angle), rev.CANSparkMax.ControlType.kPosition)
        self.angle_pid_last_reference = self.clamp_angle(pid_angle + self.rel_to_corrected_angle_adjustment)

        # ---------------------------------
        #pid_angle = angle + self.rel_to_corrected_angle_adjustment
        
        #rel_position = self.angle_motor_encoder.getPosition() 
        #adjustment = shortest_angle_difference(rel_position + self.rel_to_corrected_angle_adjustment, pid_angle)
        #self.angle_pid.setReference(self.clamp_angle(rel_position + adjustment), rev.CANSparkMax.ControlType.kPosition)
        #self.angle_pid_last_reference = self.clamp_angle(rel_position + adjustment)

    @staticmethod
    def init_motor(motor: rev.CANSparkMax, config: MotorConfig):
        motor.restoreFactoryDefaults()
        motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        motor.setInverted(config.inverted)
    
    def init_physical(self, physical_config: PhysicalConfig):
        self.angle_motor.setSmartCurrentLimit(int(physical_config.current_limit.angle))
        self.drive_motor.setSmartCurrentLimit(int(physical_config.current_limit.drive))
    
    def init_rotation(self) -> bool:
        '''Returns true if the wheel is in the correct position, false if it needs to be adjusted'''
        if self.config.encoder.offset is not None:
            required_absolute_adjustment = self.config.encoder.offset #shortest_angle_difference(self.angle_absolute_encoder.getPosition(), self.angle_absolute_encoder.getPosition() - self.config.encoder.offset)
            self.angle_motor_encoder.setPosition(self.angle_absolute_encoder.getPosition())
            #self.rel_to_absolute_angle_adjustment = shortest_angle_difference(self.clamp_angle(self.angle_motor_encoder.getPosition()), self.angle_absolute_encoder.getPosition())
            self.rel_to_absolute_angle_adjustment = 0
            self.rel_to_corrected_angle_adjustment = self.rel_to_absolute_angle_adjustment + required_absolute_adjustment
        return True
    
    def clamp_angle(self, angle: float) -> float:
        '''Clamp the angle to the range of the absolute encoder'''
        return angle % (math.pi * 2.0)
    
 