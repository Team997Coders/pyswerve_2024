import math
import commands2
import wpilib
import wpimath.controller
import wpimath.trajectory

import config
import hardware
from config import FeedForwardConfig
import swerve
from typing import Callable
import math_help
from wpilib import SmartDashboard


class TargetTracker(commands2.ProfiledPIDSubsystem):
    """A subsystem that tracks a target angle using a PIDController and a feedforward controller.
       Set a requested angle with the target property, and the hardware will move to track the target angle.
       Read the velocity property to determine how fast the hardware must move to reach the target angle."""

    _get_angle_measurement: Callable[[], float]  # Call this function to get the current heading in radians
    _get_chassis_angle_velocity_measurement: Callable[
        [], float]  # Call this function to get the current heading velocity in radians
    _target: float  # The target angle for the target tracker.  Do not set this outside of the target property accessors
    _desired_velocity: float  # How fast we want the hardware moving in radians to track the target.  The combination of PID and feedforward output.
    _angle_pid: wpimath.controller.ProfiledPIDController  # The PID controller for the target tracker
    _pid_config: config.ProfiledPIDConfig  # The PID configuration for the target tracker
    _ff_config: FeedForwardConfig  # The feedforward configuration for the target tracker
    _last_update_time: float  # The last time the target tracker sent updates to SmartDashboard
    _feedforward_component: float = 0  # The feedforward component of the most recent output
    _pid_component: float = 0  # The PID component of the most recent output

    def __init__(self,
                 get_chassis_angle_measurement: Callable[[], float],
                 get_chassis_angle_velocity_measurement: Callable[[], float],
                 angle_pid_config: config.ProfiledPIDConfig,
                 feedforward_config: FeedForwardConfig | None,
                 initial_angle: float = 0):
        """
        Create a new TargetTracker
        :param get_chassis_angle_measurement: A function that returns the current angle of the chassis in radians
        :param get_chassis_angle_velocity_measurement: A function that returns the current angular velocity of the chassis in radians
        :param angle_pid_config: The PID configuration for the target tracker
        :param feedforward_config: The feedforward configuration for the target tracker
        :param initial_angle: The initial target angle of the chassis in radians
        """
        self._angle_pid = hardware.create_profiled_pid_radians(angle_pid_config)
        super().__init__(self._angle_pid,
                         initial_angle)  # Sending a duplicate of angle_pid, but simpler to understand where pid is
        self._ff_config = feedforward_config
        self._pid_config = angle_pid_config
        self._get_angle_measurement = get_chassis_angle_measurement
        self._get_chassis_angle_velocity_measurement = get_chassis_angle_velocity_measurement
        self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(kS=feedforward_config.kS,
                                                                            kA=feedforward_config.kA,
                                                                            kV=feedforward_config.kV)
        self.target = initial_angle
        self._last_update_time = wpilib.Timer.getFPGATimestamp()

    def getMeasurement(self) -> float:
        """
        Get the angle from the gyro and ensure it falls in the expected range for the PID.
        WPI docs indicate software pids become undefined when passed values outside the wrapping range
        """
        return math_help.wrap_angle(self._get_angle_measurement(), min_val=self._pid_config.wrapping.min)

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

    def useOutput(self, output: float, setpoint: wpimath.trajectory.TrapezoidProfile.State):
        """Use the output from the controller object."""
        self._pid_component = output
        self._desired_velocity = output

        if self._ff_config is not None:
            self._feedforward_component = self.calculate_feedforward(setpoint.velocity)
            self._desired_velocity += self._feedforward_component
 
    def calculate_feedforward(self, pid_velocity: float):
        """Calculate the feedforward value for the target tracker"""
        chassis_velocity = self._get_chassis_angle_velocity_measurement()
        return self._feedforward.calculate(currentVelocity=chassis_velocity,
                                           nextVelocity=pid_velocity,
                                           dt=0.02)

    # def periodic(self):
    #     gyro_radians = math_help.wrap_angle(self._get_chassis_angle_measurement(), -math.pi)
    #     measured_chassis_speed = self._get_chassis_angle_velocity_measurement()
    #     pid_output = self._angle_pid.calculate(gyro_radians, self._desired_robot_heading)
    #     chassis_velocity = measured_chassis_speed.omega
    #     pid_velocity = self._angle_pid.getSetpoint().desired_velocity
    #     ff_value = self._feedforward.calculate(currentVelocity=chassis_velocity, nextVelocity=pid_velocity, dt=0.02)
    #
    #     theta_change = pid_output
    #
    #     ff_value = 0
    #     self.send_drive_command(x_output_value, y_output_value, theta_change + ff_value)
    #
    #     SmartDashboard.putNumberArray("outputs", [x_output_value, y_output_value, self._desired_robot_heading])
    #     SmartDashboard.putNumber("theta_change", theta_change)
    #     SmartDashboard.putNumber("Feedforward", ff_value)
    #     SmartDashboard.putNumber("desired heading", self._desired_robot_heading)
    #     SmartDashboard.putData("PID controller", self._angle_pid)
    #     SmartDashboard.putBoolean("at goal", self._angle_pid.atGoal())
    #     SmartDashboard.putBoolean("at internal setpoint", self._angle_pid.atSetpoint())
    #     SmartDashboard.putNumber("gyro", self._swerve_drive.gyro_angle_degrees)
    #     SmartDashboard.putNumber("gyro_radians", gyro_radians)
    #
    #     # pid_value = SmartDashboard.getData("PID controller")  # type: ProfiledPIDController
    #     # self._angle_pid.setP(pid_value.)
