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
import wpimath.kinematics as kinematics
import wpimath.geometry as geom
from typing import Callable
import commands2
import commands2.button
import subsystems


class SetSwerveModuleAngles(commands2.InstantCommand):
    """Tell the swerve drive to set the angle of each module.  This is useful for
       running calibration and test routines such as SysID that only move drive motors"""
    _swerve_drive: swerve.SwerveDrive
    _angle: float

    def __init__(self, swerve_drive: swerve.SwerveDrive, angle: float):
        super().__init__()
        self._swerve_drive = swerve_drive
        self._angle = angle
        self.requirements = {swerve_drive}

    def execute(self):
        desired_states = tuple([kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(self._angle)) for m in self._swerve_drive.modules])
        self._swerve_drive.drive_with_module_states(desired_states, None) # type: ignore



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
    _target_xy: [float, float]
    _get_chassis_xy: Callable[[], tuple[float, float]]
    is_heading_reversed: bool

    def __init__(self, set_heading_goal: Callable[[float], None],
                 get_chassis_xy: Callable[[], tuple[float, float]],
                 target_xy: tuple[float, float],
                 is_heading_reversed: bool):
        super().__init__()
        self._set_heading_goal = set_heading_goal
        self._get_chassis_xy = get_chassis_xy
        self._target_xy = target_xy
        self.is_heading_reversed = is_heading_reversed

    def execute(self):
        tx, ty = self._target_xy
        px, py = self._get_chassis_xy()
        heading_goal = math.atan2(ty - py, tx - px)
        if self.is_heading_reversed:
            self._set_heading_goal(heading_goal + math.pi)
        else:
            self._set_heading_goal(heading_goal)


class TwinstickHeadingSetter(commands2.Command):
    """Sets a heading based on the joystick direction"""
    set_heading_goal: Callable[[float], None]
    get_x: Callable[[], float]
    get_y: Callable[[], float]
    is_heading_inverted: bool

    def __init__(self,
                 set_heading_goal: Callable[[float], None],
                 get_x: Callable[[], float],
                 get_y: Callable[[], float],
                 is_heading_inverted: bool):
        super().__init__()
        self.set_heading_goal = set_heading_goal
        self.get_x = get_x
        self.get_y = get_y
        self.is_heading_inverted = is_heading_inverted

    def execute(self):
        try:
            x = self.get_x()
            y = self.get_y()
            if x == 0 and y == 0:
                return
            else:
                heading = geom.Rotation2d(-x, y).radians()
                if self.is_heading_inverted:
                    self.set_heading_goal(heading + math.pi)
                else:
                    self.set_heading_goal(heading)
        except:
            pass


class FlipHeading(commands2.InstantCommand):
    def __init__(self, twinstick_heading_setter: TwinstickHeadingSetter, set_target: SetTarget):
        super().__init__()
        self.twinstick_heading_setter = twinstick_heading_setter
        self.set_target = set_target

    def execute(self):
        self.twinstick_heading_setter.is_heading_inverted = not self.twinstick_heading_setter.is_heading_inverted
        self.set_target.is_heading_reversed = not self.set_target.is_heading_reversed
