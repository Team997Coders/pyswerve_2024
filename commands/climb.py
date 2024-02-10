from typing import Optional
import commands2
import wpilib
import wpimath
import rev
import logging
import robot_config
from math_help import Range
from config import AxisConfig
from math_help import shared
import wpimath.controller
from commands2 import Command
from subsystems.climber import Climber


class ClimberFollow(commands2.Command):
    _climber: Climber
    _arm_range: Range
    _arm_position: float

    def __init__(self, climber, config: AxisConfig, pid: wpimath.controller.PIDController, ticks_to_climb: float):
        super().__init__()
        self._climber = climber
        self._config = config
        self._pid = pid
        self._ticks_to_climb = ticks_to_climb
        self._arm_range = Range(0, 1)
        self._arm_position = 0.0

    def initialize(self):
        self._arm_range = Range(min_val=self._climber.climber_encoder, max_val=self._ticks_to_climb)


    def execute(self):
        input = self._config.controller.getRawAxis(self._config.axis_index)
        output = shared.map_input_to_output_range(input, self._config.input_range, self._arm_range)
        self._pid.setGoal(output)
        self._climber.velocity = self._pid.calculate(self._climber.velocity, self._climber.climber_encoder)
