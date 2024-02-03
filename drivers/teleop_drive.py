import sys
import wpilib
from config.driver_controls import AxisConfig
import robot_config
import math
from swerve import SwerveDrive
from debug import attach_debugger
from wpilib import SmartDashboard
from math_help import map_input_to_output_range

if __debug__ and "run" in sys.argv:
    # To enter debug mode, add the --debug flag to the 'deploy' command:
    # python -m robotpy deploy --debug
    # At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger()


class TeleopDrive:
    _swerve_drive: SwerveDrive
    _controller: wpilib.XboxController
    _x_config: AxisConfig
    _y_config: AxisConfig
    _theta_config: AxisConfig

    def __init__(self, swerve_drive, controller: wpilib.Joystick | wpilib.XboxController, x_config: AxisConfig, y_config: AxisConfig, theta_config: AxisConfig):
        self._swerve_drive = swerve_drive
        self.controller = controller
        self._x_config = x_config
        self._y_config = y_config
        self._theta_config = theta_config

    def do_input(self):

        x_input_value = self.controller.getRawAxis(self._x_config.axis_index)
        y_input_value = self.controller.getRawAxis(self._y_config.axis_index)
        theta_input_value = self.controller.getRawAxis(self._theta_config.axis_index)
        x_output_value = map_input_to_output_range(x_input_value, self._x_config.input_range, self._x_config.output_range)
        y_output_value = map_input_to_output_range(y_input_value, self._y_config.input_range, self._y_config.output_range)
        theta_output_value = map_input_to_output_range(theta_input_value, self._theta_config.input_range, self._theta_config.output_range)

        requested_speed = math.sqrt(x_output_value ** 2 + y_output_value ** 2)

        # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
        if requested_speed > robot_config.physical_properties.max_drive_speed:
            scale_factor = robot_config.physical_properties.max_drive_speed / requested_speed
            x_output_value *= scale_factor
            y_output_value *= scale_factor

        SmartDashboard.putNumberArray("outputs", [x_output_value, y_output_value, theta_output_value])
        self.send_drive_command(x_output_value, y_output_value, theta_output_value)

    def send_drive_command(self, vx: float, vy: float, theta: float):

        velocity = math.sqrt(
            self._swerve_drive.measured_chassis_speed.vx ** 2 + self._swerve_drive.measured_chassis_speed.vy ** 2)

        if velocity < 0.1 and theta == 0 and vx == 0 and vx == 0:
            self._swerve_drive.lock_wheels()
        else:
            self._swerve_drive.drive(-vx, -vy, -theta, None)
