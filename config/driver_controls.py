from typing import NamedTuple

import wpilib

from math_helper import Range


class DriverControlsConfig(NamedTuple):
    x_deadband: Range
    y_deadband: Range
    theta_deadband: Range


class AxisConfig(NamedTuple):
    input_range: Range
    output_range: Range
    # controller that the axis is on
    controller: wpilib.XboxController | wpilib.Joystick
    # index of axis on the controller
    axis_index: int