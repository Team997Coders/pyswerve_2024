from typing import NamedTuple

import enum
from math_help import Range
import wpilib
import commands2.button


class ControllerType(enum.IntEnum):
    Joystick = 0
    XBox = 1
    PS4 = 2
    PS5 = 3


class DriverControlsConfig(NamedTuple):
    x_deadband: Range
    y_deadband: Range
    theta_deadband: Range


class AxisConfig(NamedTuple):
    #Range we wamt to allow from controller input, absolute value is used and then negated if input was negative
    deadband: Range
    # Output range of the axis, should be in units meaningful to the robot
    output_range: Range
    input_range: Range = Range(0, 1)


class ControllerKey(NamedTuple):
    """Used to identify a controller"""
    controller_type: ControllerType
    controller_index: int


class ControllerConfig(NamedTuple):
    controller_key: ControllerKey  # The controller type and index
    axes: dict[int, AxisConfig]  # The axes on the controller we are configuring
