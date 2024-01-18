import abc
import wpilib
import rev
import math
import logging
from config import *
from wpilib import SmartDashboard as sd
from math_help import shortest_angle_difference
import wpimath.geometry as geom
import wpimath.kinematics as kinematics

class ISwerveModule(abc.ABC):
    @abc.abstractproperty
    def location(self) -> geom.Translation2d:
        '''Location of the module relative to robot center in meters'''
        raise NotImplementedError()

    @abc.abstractproperty
    def velocity(self) -> float:
        '''Velocity of the drive motor in meters per second'''
        raise NotImplementedError()

    @velocity.setter
    @abc.abstractmethod
    def velocity(self, meters_per_sec: float):
        '''Velocity of the drive motor in meters per second'''
        raise NotImplementedError()  
    
    @abc.abstractproperty
    def angle(self) -> float:
        '''Rotation of the module in radians'''
        raise NotImplementedError()
    
    @angle.setter
    @abc.abstractmethod
    def angle(self, angle: float):
        '''Rotation of the module in radians'''
        raise NotImplementedError()
    
    @abc.abstractproperty
    def position(self) -> kinematics.SwerveModulePosition:
        raise NotImplementedError()
    
    @abc.abstractproperty
    def rotation2d(self) -> geom.Rotation2d:
        '''Current rotation of the module'''
        raise NotImplementedError()
    
    @abc.abstractproperty
    def desired_state(self) -> kinematics.SwerveModuleState:
        '''The desired state of the module'''
        raise NotImplementedError()
    
    @desired_state.setter
    @abc.abstractmethod
    def desired_state(self, value: kinematics.SwerveModuleState):
        '''The desired state of the module'''
        raise NotImplementedError()
    
    @abc.abstractproperty
    def measured_state(self) -> kinematics.SwerveModuleState:
        '''The measured state of the module based on sensor inputs'''
        raise NotImplementedError() 
    
    @abc.abstractmethod
    def initialize(self) -> bool:
        '''Initialize the swerve module.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True.'''
        raise NotImplementedError()

class SwerveModule(ISwerveModule):
    id: module_position

    drive_motor: rev.CANSparkMax
    angle_motor: rev.CANSparkMax

    drive_motor_encoder: rev.SparkRelativeEncoder
    angle_motor_encoder: rev.SparkRelativeEncoder

    drive_pid : rev.SparkMaxPIDController
    angle_pid : rev.SparkMaxPIDController

    absolute_encoder: rev.AbsoluteEncoder
 
    angle_pid_last_reference: float

    rel_to_absolute_angle_adjustment: float | None # How far we need to adjust the pid reference to get the absolute encoder to read the correct angle
    rel_to_corrected_angle_adjustment: float | None # How far we need to adjust the pid reference to get the angle relative to robot chassis
    config: SwerveModuleConfig

    _desired_state: kinematics.SwerveModuleState

    logger: logging.Logger # Used to write log messages to driver station

    def __init__(self, id: module_position, module_config: SwerveModuleConfig, physical_config: PhysicalConfig, logger: logging.Logger):
        self.id = id 
        self.logger = logger.getChild(str(id))
        self._desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(0))
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
        self.angle_pid.setD(0)

        self.drive_pid.setP(0.2)
        self.drive_pid.setI(0)
        self.drive_pid.setD(0.0)
        
        self.drive_motor_encoder.setPositionConversionFactor(1.0 / physical_config.gear_ratio.drive)
        self.drive_motor_encoder.setVelocityConversionFactor((1.0 / physical_config.gear_ratio.drive * (physical_config.wheel_diameter_cm / 100 * math.pi)) / 60.0) # convert from rpm to revolutions/sec
        self.drive_pid.setFeedbackDevice(self.drive_motor_encoder)
        
        self.angle_motor_encoder.setPositionConversionFactor((2.0 * math.pi) / physical_config.gear_ratio.angle)
        self.angle_motor_encoder.setVelocityConversionFactor((2.0 * math.pi) / (physical_config.gear_ratio.angle * 60.0)) # convert from rpm to revolutions/sec
        self.angle_absolute_encoder = self.angle_motor.getAbsoluteEncoder(encoderType=rev.SparkAbsoluteEncoder.Type.kDutyCycle)
        
        if module_config.encoder.conversion_factor is not None:
            self.angle_absolute_encoder.setPositionConversionFactor(module_config.encoder.conversion_factor)
            self.angle_motor_encoder.setPosition(self.angle_absolute_encoder.getPosition())

        self.angle_pid.setFeedbackDevice(self.angle_absolute_encoder) 
        self.angle_pid.setPositionPIDWrappingEnabled(True)
        self.angle_pid.setPositionPIDWrappingMinInput(0)

        #If you do not want the optimization of shortening the angle rotation to minimum distance, change PID wrapping to 2 * PI
        self.angle_pid.setPositionPIDWrappingMaxInput(math.pi)
        # self.angle_pid.setFeedbackDevice(self.angle_absolute_encoder)

        self.angle_motor.burnFlash()
        self.drive_motor.burnFlash()
  
    def __str__(self):
        return f"SwerveModule {str(self.id)}"
    
    @property
    def location(self) -> geom.Translation2d:
        '''Get the location of the wheel in meters'''
        return geom.Translation2d(self.config.location[0], self.config.location[1])

    @property
    def velocity(self) -> float:
        '''Get the velocity of the drive motor in meters per second'''
        return self.drive_motor_encoder.getVelocity()
    
    @velocity.setter
    def velocity(self, meters_per_sec: float):
        '''Set the velocity of the drive motor in meters per second'''
        self._desired_state = kinematics.SwerveModuleState(meters_per_sec, self._desired_state.angle)
        self.drive_pid.setReference(meters_per_sec, rev.CANSparkMax.ControlType.kVelocity)
    
    @property
    def raw_position(self) -> float:
        return self.drive_motor_encoder.getPosition()
    
    @property
    def angle(self) -> float:
        '''Get the angle of the wheel in radians'''
        #angle = self.angle_absolute_encoder.getPosition()
        #if not self.drive_motor.getLastError() == rev.REVLibError.kOk:
        angle = self.angle_absolute_encoder.getPosition()
        #    self.logger.info(f"Absolute encoder read error on {str(self.id)} module")
        
        return angle
    
    @angle.setter
    def angle(self, angle: float):
        '''Set the angle of the wheel in radians'''
        self._desired_state = kinematics.SwerveModuleState(self._desired_state.speed,
                                                           geom.Rotation2d(angle))
        
        # If possible, adjust the offset in Rev Hardware Client instead of in software
        if self.rel_to_corrected_angle_adjustment is None or self.rel_to_corrected_angle_adjustment == 0:
            self.angle_pid.setReference(self.clamp_angle(angle), rev.CANSparkMax.ControlType.kPosition)
            self.angle_pid_last_reference = self.clamp_angle(angle)
        else: 
            #Otherwise, attempt to adjust the angle in software using the provided offset
            pid_angle = angle + self.rel_to_corrected_angle_adjustment
            abs_position = self.angle_absolute_encoder.getPosition()
            adjustment = shortest_angle_difference(abs_position, pid_angle)
            self.angle_pid.setReference(self.clamp_angle(abs_position + adjustment), rev.CANSparkMax.ControlType.kPosition)
            self.angle_pid_last_reference = self.clamp_angle(abs_position + adjustment)

    def rotate_drive_wheel(self, num_rotations: float):
        '''Drive the wheel the specified number of rotations'''
        pos = self.drive_motor_encoder.getPosition()
        self.drive_pid.setReference(pos + num_rotations, rev.CANSparkMax.ControlType.kPosition)
    
    @property
    def position(self) -> kinematics.SwerveModulePosition:
        return kinematics.SwerveModulePosition(self.raw_position, self.rotation2d)
    
    @property
    def rotation2d(self) -> geom.Rotation2d:
        return geom.Rotation2d(self.angle)
     
    @property
    def measured_state(self) -> kinematics.SwerveModuleState:
        return kinematics.SwerveModuleState(self.velocity, self.rotation2d)
    
    @property
    def desired_state(self) -> kinematics.SwerveModuleState:
        return self._desired_state
    
    @desired_state.setter
    def desired_state(self, value: kinematics.SwerveModuleState):
        '''Sets the desired state of the module, optimizing for shortest path''' 
        #self._desired_state = value
        self._desired_state = kinematics.SwerveModuleState.optimize(value, self.rotation2d)
        
        self.angle = self._desired_state.angle.radians()
        self.velocity = self._desired_state.speed

    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        return radians * 180 / math.pi

    def report_to_dashboard(self): 
        sd.putNumber(f"Drive {int(self.id)} Position", self.raw_position)
        sd.putNumber(f"Drive {int(self.id)} Velocity", self.velocity)
        sd.putNumber(f"Drive {int(self.id)} Tgt Vel", self._desired_state.speed)
        sd.putNumber(f"Angle {int(self.id)} Position", self.radians_to_degrees(self.angle / math.pi))
        sd.putNumber(f"Angle {int(self.id)} Absolute", self.radians_to_degrees(self.angle_absolute_encoder.getPosition()))
        sd.putNumber(f"Angle {int(self.id)} Velocity", self.angle_motor_encoder.getVelocity())
        sd.putNumber(f"Angle PID {int(self.id)} Reference", self.radians_to_degrees(self.angle_pid_last_reference))
        #sd.putNumber(f"Rel to Abs {int(self.id)} adjust", self.radians_to_degrees(self.rel_to_absolute_angle_adjustment))
        #sd.putNumber(f"Rel to Chassis {int(self.id)} adjust", self.radians_to_degrees(self.rel_to_corrected_angle_adjustment))
 

    @staticmethod
    def init_motor(motor: rev.CANSparkMax, config: MotorConfig):
        motor.restoreFactoryDefaults()
        motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
        motor.setInverted(config.inverted)
    
    def init_physical(self, physical_config: PhysicalConfig):

        if physical_config.current_limit.angle is not None:
            self.angle_motor.setSmartCurrentLimit(physical_config.current_limit.angle)

        if physical_config.current_limit.drive is not None:
            self.drive_motor.setSmartCurrentLimit(physical_config.current_limit.drive)
        
        if physical_config.ramp_rate.drive is not None:
            self.drive_motor.setOpenLoopRampRate(physical_config.ramp_rate.drive)
            self.drive_motor.setClosedLoopRampRate(physical_config.ramp_rate.drive)
        
        if physical_config.ramp_rate.angle is not None:
            self.angle_motor.setOpenLoopRampRate(physical_config.ramp_rate.angle)
            self.angle_motor.setClosedLoopRampRate(physical_config.ramp_rate.angle) 
    
    def initialize(self) -> bool:
        '''Returns true if the wheel is in the correct position, false if it needs to be adjusted'''
        if self.config.encoder.offset is not None:
            required_absolute_adjustment = self.config.encoder.offset #shortest_angle_difference(self.angle_absolute_encoder.getPosition(), self.angle_absolute_encoder.getPosition() - self.config.encoder.offset)
            self.angle_motor_encoder.setPosition(self.angle_absolute_encoder.getPosition())
            self.rel_to_absolute_angle_adjustment = shortest_angle_difference(self.clamp_angle(self.angle_motor_encoder.getPosition()), self.angle_absolute_encoder.getPosition())
            self.rel_to_corrected_angle_adjustment = self.rel_to_absolute_angle_adjustment + required_absolute_adjustment
        else:
            self.rel_to_absolute_angle_adjustment = None
            self.rel_to_absolute_angle_adjustment = None

        return True
    
    def clamp_angle(self, angle: float) -> float:
        '''Clamp the angle to the range of the absolute encoder'''
        return angle % (math.pi * 2.0)
     
    
 