import sys
import wpilib
import wpimath

from config.driver_controls import AxisConfig
from robots import crescendo
import math
import math_help
from swerve import SwerveDrive
from debug import attach_debugger
from wpilib import SmartDashboard
from math_help import map_input_to_output_range
import wpimath.controller
import wpimath.geometry as geom
import commands2.button

if __debug__ and "run" in sys.argv:
    # To enter debug mode, add the --debug flag to the 'deploy' command:
    # python -m robotpy deploy --debug
    # At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger()


class TwinStickTeleopDrive:
    _swerve_drive: SwerveDrive
    _x_config: AxisConfig
    _y_config: AxisConfig
    _rotx_config: AxisConfig
    _roty_config: AxisConfig

    # https://docs.wpilib.org/en/stable/docs/software/commandbased/pid-subsystems-commands.html#pid-control-through-pidsubsystems-and-pidcommands
    # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html#introduction-to-pid
    # https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/profiled-pidcontroller.html
    _angle_pid: wpimath.controller.ProfiledPIDController

    _desired_robot_heading: float  # The last valid robot heading that was requested
    _feedforward: wpimath.controller.SimpleMotorFeedforwardMeters

    def __init__(self, swerve_drive, x_config: AxisConfig, y_config: AxisConfig, rotx_config: AxisConfig,
                 roty_config: AxisConfig, angle_pid: wpimath.controller.PIDController):
        self._swerve_drive = swerve_drive
        self._x_config = x_config
        self._y_config = y_config
        self._rotx_config = rotx_config
        self._roty_config = roty_config
        self._desired_robot_heading = self._swerve_drive.gyro_angle_radians
        self._feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            kS=0.0,
            kV=0.01,
            kA=0.001
        )

        self._angle_pid = angle_pid

    @staticmethod
    def _input_to_output(axis_config: AxisConfig):
        "Convert a controller input value into desired robot scale output values"
        controller_value = axis_config.controller.getRawAxis(axis_config.axis_index)
        return map_input_to_output_range(controller_value, axis_config.input_range, axis_config.output_range)

    @classmethod


    def drive(self):
        x_output_value = self._input_to_output(self._x_config)
        y_output_value = self._input_to_output(self._y_config)
        rotx_output_value = self._input_to_output(self._rotx_config)
        roty_output_value = self._input_to_output(self._roty_config)

        rot_magnitude = math.sqrt((rotx_output_value ** 2) + (roty_output_value ** 2))

        if rotx_output_value != 0 or roty_output_value != 0 and rot_magnitude > 0.5:
            rot = geom.Rotation2d(-rotx_output_value, -roty_output_value)
            _updated_desired_robot_heading = rot.radians()
            if abs(self._desired_robot_heading - _updated_desired_robot_heading) > (math.pi / 90):
                # self._angle_pid.reset(self._swerve_drive.pose.rotation().radians())
                self._desired_robot_heading = _updated_desired_robot_heading
                self._angle_pid.setGoal(self._desired_robot_heading)

        gyro_radians = math_help.wrap_angle(self._swerve_drive.gyro_angle_radians, -math.pi)
        measured_chassis_speed = self._swerve_drive.measured_chassis_speed
        pid_output = self._angle_pid.calculate(gyro_radians, self._desired_robot_heading)
        chassis_velocity = measured_chassis_speed.omega
        pid_velocity = self._angle_pid.getSetpoint().velocity
        ff_value = self._feedforward.calculate(currentVelocity=chassis_velocity, nextVelocity=pid_velocity, dt=0.02)

        theta_change = pid_output

        ff_value = 0
        self.send_drive_command(x_output_value, y_output_value, theta_change + ff_value)

        SmartDashboard.putNumberArray("outputs", [x_output_value, y_output_value, self._desired_robot_heading])
        SmartDashboard.putNumber("theta_change", theta_change)
        SmartDashboard.putNumber("Feedforward", ff_value)
        SmartDashboard.putNumber("desired heading", self._desired_robot_heading)
        SmartDashboard.putData("PID controller", self._angle_pid)
        SmartDashboard.putBoolean("at goal", self._angle_pid.atGoal())
        SmartDashboard.putBoolean("at internal setpoint", self._angle_pid.atSetpoint())
        SmartDashboard.putNumber("gyro", self._swerve_drive.gyro_angle_degrees)
        SmartDashboard.putNumber("gyro_radians", gyro_radians)

        #pid_value = SmartDashboard.getData("PID controller")  # type: ProfiledPIDController
        #self._angle_pid.setP(pid_value.)

    def send_drive_command(self, vx: float, vy: float, theta: float):

        velocity = math.sqrt(
            self._swerve_drive.measured_chassis_speed.vx ** 2 + self._swerve_drive.measured_chassis_speed.vy ** 2)

        if velocity < 0.1 and theta == 0 and vx == 0 and vx == 0:
            self._swerve_drive.lock_wheels()
        else:
            self._swerve_drive.drive(-vx, -vy, theta, None)