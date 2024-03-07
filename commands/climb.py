import commands2
from math_help import Range
from config import AxisConfig
from math_help import shared
import wpimath.controller
from subsystems.climber import Climber
from typing import Callable


class ClimberFollow(commands2.Command):
    _climber: Climber

    def __init__(self, climber, height_getter: Callable[[], float]):
        super().__init__()
        self._climber = climber
        self._climb_input = height_getter()

    def execute(self):
        self._climber.position = self._climb_input


class ClimberUp(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):
        self._climber.speed = -0.5


class ClimberDown(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):
        self._climber.speed = 0.5


class ClimberStop(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):
        self._climber.speed = 0
