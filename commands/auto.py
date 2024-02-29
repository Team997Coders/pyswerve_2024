import sys
import wpilib
import wpimath

import swerve
import config
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
from typing import Callable, Tuple
import commands2
import commands2.button
import subsystems
import hardware
from subsystems import axis_position_control


class DeadReckonX(commands2.Command):
    swerve: SwerveDrive
    x_distance: float

    def __init__(self, swerve_drive: SwerveDrive, x_distance: float):
        super().__init__()
        self.swerve_drive = swerve_drive
        self.x_distance = x_distance

    def execute(self):
        if self.x_distance > 0:
            self.swerve_drive.drive_set_distance(self.x_distance, 0)
        else:
            self.swerve_drive.drive_set_distance(self.x_distance, 180)


class DeadReckonY(commands2.Command):
    swerve: SwerveDrive
    y_distance: float

    def __init__(self, swerve_drive: SwerveDrive, y_distance: float):
        super().__init__()
        self.swerve_drive = swerve_drive
        self.y_distance = y_distance

    def execute(self):
        if self.y_distance > 0:
            self.swerve_drive.drive_set_distance(self.y_distance, 270)
        else:
            self.swerve_drive.drive_set_distance(self.y_distance, 90)


class GotoXYTheta(commands2.Command):
    """Moves robot to specific XYZ position relative to startup pose on field"""
    _swerve_drive: swerve.SwerveDrive
    _destination_xyt: tuple[float, float, float]
    _x_axis_pid: subsystems.AxisPositionControl
    _y_axis_pid = subsystems.AxisPositionControl
    _theta_axis_pid = subsystems.ChassisHeadingControl

    @property
    def target(self) -> tuple[float, float, float]:
        return self._destination_xyt

    @target.setter
    def target(self, value: tuple[float, float, float]):
        self._destination_xyt = value
        self._x_axis_pid.setGoal(value[0])
        self._y_axis_pid.setGoal(value[1])
        self._theta_axis_pid.setGoal(value[2])

    def __init__(self,
                 swerve_drive: SwerveDrive,
                 destination_xy_theta: Tuple[float, float, float],
                 x_axis_pid: subsystems.AxisPositionControl,
                 y_axis_pid: subsystems.AxisPositionControl,
                 theta_axis_pid: subsystems.ChassisHeadingControl
                 ):
        super().__init__()
        self._destination_xyt = destination_xy_theta
        self._swerve_drive = swerve_drive
        self._x_axis_pid = x_axis_pid
        self._y_axis_pid = y_axis_pid
        self._theta_axis_pid = theta_axis_pid

    def initialize(self):
        self.target = self._destination_xyt

    def execute(self):
        self._swerve_drive.drive(self._x_axis_pid.desired_velocity,
                                 self._y_axis_pid.desired_velocity,
                                 self._theta_axis_pid.desired_velocity)

    def isFinished(self) -> bool:
        finished = self._x_axis_pid.atTarget() and self._y_axis_pid.atTarget() and self._theta_axis_pid.atTarget()
        if finished:
            print("Finished!")

        return finished
