import time
from typing import Callable, Any
import rev
import math
import logging
import hardware
import wpilib.sysid

import math_help
from config import ModulePosition, SwerveModuleConfig, PhysicalConfig
from wpilib import SmartDashboard as sd
from math_help import shortest_angle_difference
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
from swerve import ISwerveModule


class SwerveModule(ISwerveModule):
    _id: ModulePosition

    _drive_motor: rev.CANSparkMax
    _angle_motor: rev.CANSparkMax

    drive_motor_encoder: rev.SparkRelativeEncoder
    angle_motor_encoder: rev.SparkRelativeEncoder

    drive_pid: rev.SparkMaxPIDController
    angle_pid: rev.SparkMaxPIDController

    absolute_encoder: rev.AbsoluteEncoder

    angle_pid_last_reference: float

    rel_to_absolute_angle_adjustment: float | None  # How far we need to adjust the pid reference to get the absolute encoder to read the correct angle
    rel_to_corrected_angle_adjustment: float | None  # How far we need to adjust the pid reference to get the angle relative to robot chassis
    config: SwerveModuleConfig
    _physical_config: PhysicalConfig

    _desired_state: kinematics.SwerveModuleState

    logger: logging.Logger  # Used to write log messages to driver station

    @property
    def drive_motor(self) -> rev.CANSparkMax:
        """Don't use this unless for testing"""
        return self._drive_motor

    @property
    def angle_motor(self) -> rev.CANSparkMax:
        """Don't use this unless for testing"""
        return self._angle_motor

    @property
    def id(self) -> ModulePosition:
        """Module position ID"""
        return self._id

    def angle_pid(self) -> rev.SparkMaxPIDController:
        """This is exposed only for SmartDashboard to edit PID values"""
        return self.angle_pid

    def drive_pid(self) -> rev.SparkMaxPIDController:
        """This is exposed only for SmartDashboard to edit PID values"""
        return self.drive_pid

    def __init__(self, position: ModulePosition, module_config: SwerveModuleConfig, physical_config: PhysicalConfig,
                 logger: logging.Logger):
        self._id = position
        self._physical_config = physical_config
        self.logger = logger.getChild(str(id))
        self._desired_state = kinematics.SwerveModuleState(0, geom.Rotation2d(0))
        self.rel_to_absolute_angle_adjustment = 0
        self.rel_to_corrected_angle_adjustment = 0
        self.config = module_config
        self.angle_pid_last_reference = 0
        self._drive_motor = rev.CANSparkMax(module_config.drive_motor.id, rev.CANSparkMax.MotorType.kBrushless)
        self._angle_motor = rev.CANSparkMax(module_config.angle_motor.id, rev.CANSparkMax.MotorType.kBrushless)

        hardware.init_motor(self._drive_motor, module_config.drive_motor)
        hardware.init_motor(self._angle_motor, module_config.angle_motor)

        self.drive_pid = self._drive_motor.getPIDController()
        self.angle_pid = self._angle_motor.getPIDController()

        self.drive_motor_encoder = self._drive_motor.getEncoder()
        self.angle_motor_encoder = self._angle_motor.getEncoder()

        hardware.safe_set_rev_in_thread(self.drive_motor_encoder.setPositionConversionFactor,
                                        (1.0 / physical_config.gear_ratio.drive) *
                                        ((physical_config.wheel_diameter_cm / 100) * math.pi))
        # Use the line below to report number of rotations for wheel rotation tests
        # self.drive_motor_encoder.setPositionConversionFactor(1.0 / physical_config.gear_ratio.drive)
        hardware.safe_set_rev_in_thread(self.drive_motor_encoder.setVelocityConversionFactor,
                                        (1.0 / physical_config.gear_ratio.drive) * (
                                                (
                                                            physical_config.wheel_diameter_cm / 100) * math.pi) / 60.0)  # convert from rpm to revolutions/sec

        # Request specific angles from the PID controller 
        hardware.init_pid(self.drive_pid, module_config.drive_pid, feedback_device=self.drive_motor_encoder)

        hardware.safe_set_rev_in_thread(self.angle_motor_encoder.setPositionConversionFactor,
                                        (2.0 * math.pi) / physical_config.gear_ratio.angle)
        hardware.safe_set_rev_in_thread(self.angle_motor_encoder.setVelocityConversionFactor,
                                        (2.0 * math.pi) / (
                                                physical_config.gear_ratio.angle * 60.0))  # convert from rpm to revolutions/sec
        self.angle_absolute_encoder = self._angle_motor.getAbsoluteEncoder(
            encoderType=rev.SparkAbsoluteEncoder.Type.kDutyCycle)

        if module_config.encoder.conversion_factor is not None:
            hardware.safe_set_rev_in_thread(self.angle_absolute_encoder.setPositionConversionFactor,
                                            module_config.encoder.conversion_factor)

        # self.angle_motor_encoder(module_config.encoder.inverted)

        hardware.safe_set_rev_in_thread(self.angle_motor_encoder.setPosition,
                                        self.angle_absolute_encoder.getPosition())

        hardware.init_pid(self.angle_pid, module_config.angle_pid, feedback_device=self.angle_absolute_encoder)

        hardware.safe_set_rev_in_thread(self._angle_motor.burnFlash)
        hardware.safe_set_rev_in_thread(self._drive_motor.burnFlash)

    def stop(self):
        """Idle both motors"""
        self._angle_motor.set(0)
        self._drive_motor.set(0)
        self._desired_state = kinematics.SwerveModuleState(self.angle, self.rotation2d)

    def __str__(self):
        return f"SwerveModule {str(self.id)}"

    @property
    def location(self) -> geom.Translation2d:
        """Get the location of the wheel in meters"""
        return geom.Translation2d(self.config.location[0], self.config.location[1])

    @property
    def velocity(self) -> float:
        """Get the velocity of the drive motor in meters per second"""
        return self.drive_motor_encoder.getVelocity()

    @velocity.setter
    def velocity(self, meters_per_sec: float):
        """Set the velocity of the drive motor in meters per second"""
        self._desired_state = kinematics.SwerveModuleState(meters_per_sec, self._desired_state.angle)
        self.drive_pid.setReference(meters_per_sec, rev.CANSparkMax.ControlType.kVelocity)

    @property
    def raw_position(self) -> float:
        """Drive motor encoder position"""
        return self.drive_motor_encoder.getPosition()

    @property
    def angle(self) -> float:
        """Get the angle of the wheel in radians"""
        # angle = self.angle_absolute_encoder.getPosition()
        # if not self._drive_motor.getLastError() == rev.REVLibError.kOk:
        angle = self.angle_absolute_encoder.getPosition()
        #    self.logger.info(f"Absolute encoder read error on {str(self.id)} module")
        return angle

    @angle.setter
    def angle(self, angle: float):
        """Set the angle of the wheel in radians"""
        self._desired_state = kinematics.SwerveModuleState(self._desired_state.speed,
                                                           geom.Rotation2d(angle))

        # If possible, adjust the offset in Rev Hardware Client instead of in software
        if self.rel_to_corrected_angle_adjustment is None or self.rel_to_corrected_angle_adjustment == 0:
            self.angle_pid.setReference(math_help.wrap_angle(angle), rev.CANSparkMax.ControlType.kPosition)
            self.angle_pid_last_reference = math_help.wrap_angle(angle)
        else:
            # Otherwise, attempt to adjust the angle in software using the provided offset

            # There is almost certainly a bug in this code where the drive motor will move 
            # the wrong direction if the offset is more than 90 degrees.  That would cause
            # the shortest_angle_difference to go to 180 degrees of the exepected angle 
            # but not reverse the drive motor.  The fix is probably to check the offsets 
            # in the desired_state setter and reverse drive motors for all offsets more 
            # than +/- 90 degrees.  We do not run this configuration so haven't needed to 
            # debug it.
            pid_angle = angle + self.rel_to_corrected_angle_adjustment
            abs_position = self.angle_absolute_encoder.getPosition()
            adjustment = shortest_angle_difference(abs_position, pid_angle)
            self.angle_pid.setReference(math_help.wrap_angle(abs_position + adjustment),
                                        rev.CANSparkMax.ControlType.kPosition)
            self.angle_pid_last_reference = math_help.wrap_angle(abs_position + adjustment)

    def rotate_drive_wheel(self, num_rotations: float):
        """Drive the wheel the specified number of rotations"""
        # self.safe_set(self.drive_motor_encoder.setPosition,0)
        pos = self.drive_motor_encoder.getPosition()
        self.drive_pid.setReference(pos + num_rotations, rev.CANSparkMax.ControlType.kPosition)

    @property
    def position(self) -> kinematics.SwerveModulePosition:
        """SwerveModulePosition object representing the position of the wheel"""
        return kinematics.SwerveModulePosition(self.raw_position, self.rotation2d)

    @property
    def rotation2d(self) -> geom.Rotation2d:
        """Measured wheel rotation"""
        return geom.Rotation2d(self.angle)

    @property
    def measured_state(self) -> kinematics.SwerveModuleState:
        """Swerve module state representing the measured state of the module, not the desired state."""
        return kinematics.SwerveModuleState(self.velocity, self.rotation2d)

    @property
    def desired_state(self) -> kinematics.SwerveModuleState:
        """Where the PID controllers were last instructed to rotate the wheel and how fast to drive it"""
        return self._desired_state

    @desired_state.setter
    def desired_state(self, value: kinematics.SwerveModuleState):
        """Sets the desired state of the module, optimizing for shortest rotation path"""
        self._desired_state = math_help.optimize_state_improved(value, geom.Rotation2d(
            self.angle_absolute_encoder.getPosition()))
        self.angle = self._desired_state.angle.radians()
        self.velocity = self._desired_state.speed

    def drive_set_distance(self, meters: float, angle: float):
        """Drive the wheel a specific distance in meters.  If you use this call, desired state speed will be incorrect until
           you set desired_state again. This is because the PID controller will be driving the wheel to the specified distance."""
        self.drive_pid.setReference(meters, rev.CANSparkMax.ControlType.kPosition)
        self.angle = angle

    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        return radians * 180 / math.pi

    def report_to_dashboard(self):
        sd.putNumber(f"Drive {int(self.id)} Position", self.raw_position)
        sd.putNumber(f"Drive {int(self.id)} Velocity", self.velocity)
        sd.putNumber(f"Drive {int(self.id)} Tgt Vel", self._desired_state.speed)
        sd.putNumber(f"Angle {int(self.id)} Position", math.degrees(self.angle_motor_encoder.getPosition()))
        sd.putNumber(f"Angle {int(self.id)} Absolute", math.degrees(self.angle_absolute_encoder.getPosition()))
        sd.putNumber(f"Angle {int(self.id)} Velocity", self.angle_motor_encoder.getVelocity())
        sd.putNumber(f"Angle PID {int(self.id)} Reference", math.degrees(self.angle_pid_last_reference))
        # sd.putNumber(f"Rel to Abs {int(self.id)} adjust", self.radians_to_degrees(self.rel_to_absolute_angle_adjustment))
        # sd.putNumber(f"Rel to Chassis {int(self.id)} adjust", self.radians_to_degrees(self.rel_to_corrected_angle_adjustment)

    def initialize(self) -> bool:
        """Returns true if the wheel is in the correct position, false if it needs to be adjusted"""
        if self.config.encoder.offset is not None:
            required_absolute_adjustment = self.config.encoder.offset  # shortest_angle_difference(self.angle_absolute_encoder.getPosition(), self.angle_absolute_encoder.getPosition() - self.config.encoder.offset)
            self.angle_motor_encoder.setPosition(self.angle_absolute_encoder.getPosition())
            self.rel_to_absolute_angle_adjustment = shortest_angle_difference(
                math_help.wrap_angle(self.angle_motor_encoder.getPosition()), self.angle_absolute_encoder.getPosition())
            self.rel_to_corrected_angle_adjustment = self.rel_to_absolute_angle_adjustment + required_absolute_adjustment
        else:
            self.rel_to_absolute_angle_adjustment = None
            self.rel_to_absolute_angle_adjustment = None

        return True

    def drive_at_voltage(self, voltage: float):
        """Drive the wheel at the specified voltage"""
        self.angle = 0  # Safety to ensure motors are all pointed in the same direction
        self._drive_motor.setVoltage(voltage)

    def log_to_sysid(self, log: wpilib.sysid.SysIdRoutineLog.MotorLog):
        log.velocity(self.velocity)
        log.position(self.position.distance)
        log.voltage(self.drive_motor.getAppliedOutput())
        log.current(self.drive_motor.getOutputCurrent())
