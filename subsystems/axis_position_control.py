import math
import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory
import config
import hardware
from config import FeedForwardConfig
from typing import Callable
import math_help


class AxisPositionControl(commands2.ProfiledPIDSubsystem):
    """A subsystem that tracks a target position on an axis using a PIDController and a feedforward controller.
       Set a requested position with the target property, and the hardware will move to track the target angle.
       Read the velocity property to determine how fast the hardware must move to reach the target angle."""

    _get_chassis_velocity_measurement: Callable[
        [], float]  # Call this function to get the current heading velocity in radians
    _desired_velocity: float  # How fast we want the hardware moving in m/s to reach the target.  The combination of PID and feedforward output.
    _position_pid: wpimath.controller.ProfiledPIDController  # The PID controller for the target tracker
    _pid_config: config.ProfiledPIDConfig  # The PID configuration for the target tracker
    _ff_config: FeedForwardConfig  # The feedforward configuration for the target tracker
    _last_update_time: float  # The last time the target tracker sent updates to SmartDashboard
    _feedforward_component: float = 0  # The feedforward component of the most recent output
    _pid_component: float = 0  # The PID component of the most recent output

    @property
    def pid(self) -> wpimath.controller.ProfiledPIDController:
        return self._position_pid

    def __init__(self,
                 get_chassis_position_measurement: Callable[[], float],
                 get_chassis_velocity_measurement: Callable[[], float],
                 pid_config: config.ProfiledPIDConfig,
                 feedforward_config: FeedForwardConfig | None,
                 initial_position: float = 0):
        """
        Create a new TargetTracker
        :param get_chassis_angle_measurement: A function that returns the current angle of the chassis in radians
        :param get_chassis_angle_velocity_measurement: A function that returns the current angular velocity of the chassis in radians
        :param angle_pid_config: The PID configuration for the target tracker
        :param feedforward_config: The feedforward configuration for the target tracker
        :param initial_angle: The initial target angle of the chassis in radians
        """
        self._position_pid = hardware.create_profiled_pid(pid_config)
        super().__init__(self._position_pid,
                         initial_position)  # Sending a duplicate of angle_pid, but simpler to understand where pid is
        self._ff_config = feedforward_config
        self._pid_config = pid_config
        self._get_position_measurement = get_chassis_position_measurement
        self._get_chassis_velocity_measurement = get_chassis_velocity_measurement
        if feedforward_config is not None:
            self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(kS=feedforward_config.kS,
                                                                                kA=feedforward_config.kA,
                                                                                kV=feedforward_config.kV)
        else:
            self._feedforward = None

        self._desired_velocity = 0
        self._target = initial_position
        self._last_update_time = wpilib.Timer.getFPGATimestamp()

    def getMeasurement(self) -> float:
        """
        Get the angle from the gyro and ensure it falls in the expected range for the PID.
        WPI docs indicate software pids become undefined when passed values outside the wrapping range
        """
        return self._get_position_measurement()

    def set_current_position(self, value: float):
        """Reset the PID to be at the value"""
        self._position_pid.reset(value)

    @property
    def desired_velocity(self) -> float:
        """The current velocity the target tracker would like the chassis to be moving"""
        return self._desired_velocity

    @property
    def target(self) -> float:
        """The target angle for the target tracker"""
        return self._target

    @target.setter
    def target(self, value: float):
        if self._target == value:
            return

        self._target = value
        self.setGoal(value)

    def setTarget(self, value: float):
        if self._target == value:
            return

        self._target = value
        self.setGoal(value)

    def atTarget(self) -> bool:
        return self._position_pid.atGoal()

    def useOutput(self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State):
        """Use the output from the controller object."""
        self._pid_component = output
        self._desired_velocity = output

        if self._ff_config is not None:
            self._feedforward_component = self.calculate_feedforward(setpoint.velocity)
            self._desired_velocity += self._feedforward_component

    def calculate_feedforward(self, pid_velocity: float):
        """Calculate the feedforward value for the target tracker"""
        chassis_velocity = self._get_chassis_velocity_measurement()
        return self._feedforward.calculate(currentVelocity=chassis_velocity,
                                           nextVelocity=pid_velocity,
                                           dt=0.02)
