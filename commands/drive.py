import sys
import wpilib
import wpimath

import swerve
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
from typing import Callable
import commands2
import commands2.button
import subsystems


class Drive(commands2.Command):
    _swerve_drive: swerve.SwerveDrive
    _feedforward: wpimath.controller.SimpleMotorFeedforwardMeters
    get_x: Callable[[], float]
    get_y: Callable[[], float]
    get_theta: Callable[[], float]

    def __init__(self, swerve_drive: swerve.SwerveDrive,
                 get_x: Callable[[], float],
                 get_y: Callable[[], float],
                 get_theta: Callable[[], float]):
        super().__init__()
        self._swerve_drive = swerve_drive
        self.get_x = get_x
        self.get_y = get_y
        self.get_theta = get_theta
        self.requirements = {swerve_drive}  # Nobody can talk to the swerve while this command is active

    def execute(self):
        x = self.get_x()
        y = self.get_y()
        theta = self.get_theta()
        self.send_drive_command(x, y, theta)

    def send_drive_command(self, vx: float, vy: float, vtheta: float):

        chassis_speeds = self._swerve_drive.measured_chassis_speed
        velocity = math.sqrt(chassis_speeds.vx ** 2 + chassis_speeds.vy ** 2)

        if velocity < 0.1 and vtheta == 0 and vx == 0 and vx == 0:
            self._swerve_drive.lock_wheels()
        else:
            self._swerve_drive.drive(vx, vy, vtheta)


class SetTarget(commands2.Command):
    _set_heading_goal: Callable[[float], None]
    _get_target_xy: Callable[[], tuple[float, float]]
    _get_chassis_xy: Callable[[], tuple[float, float]]

    def __init__(self, set_heading_goal: Callable[[float], None],
                 get_chassis_xy: Callable[[], tuple[float, float]],
                 get_target_xy: Callable[[], tuple[float, float]]):
        super().__init__()
        self._set_heading_goal = set_heading_goal
        self._get_chassis_xy = get_chassis_xy
        self._get_target_xy = get_target_xy

    def execute(self):
        tx, ty = self._get_target_xy()
        px, py = self._get_chassis_xy()
        heading_goal = math.atan2(ty - py, tx - px)
        self._set_heading_goal(heading_goal)


class TwinstickHeadingSetter(commands2.Command):
    """Sets a heading based on the joystick direction"""
    set_heading_goal: Callable[[float], None]
    get_x: Callable[[], float]
    get_y: Callable[[], float]

    def __init__(self,
                 set_heading_goal: Callable[[float], None],
                 get_x: Callable[[], float],
                 get_y: Callable[[], float]):
        super().__init__()
        self.set_heading_goal = set_heading_goal
        self.get_x = get_x
        self.get_y = get_y

    def execute(self):
        x = self.get_x()
        y = self.get_y()
        heading = geom.Rotation2d(-x, y).radians()
        self.set_heading_goal(heading)
