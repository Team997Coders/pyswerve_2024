import commands2
from math_help import Range
from config import AxisConfig
from math_help import shared
import wpimath.controller
from subsystems.climber import Climber


class ClimberFollow(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber, height_getter: callable[[], float]):
        super().__init__()
        self._climber = climber
        self._climb_input = height_getter()

    def execute(self):
        self._climber.position = self._climb_input
