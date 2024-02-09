from typing import Optional
import commands2
import wpilib
import wpimath
import rev
import logging
import robot_config
from robot_config import climber_constants
from commands2 import Command
from subsystems.climber import Climber

class climb(commands2.command):
    def __init__(self, climber, climber_pid):
        self._climber = climber
        self._climber_pid = climber_pid

    def initialize(self):

    def execute(self):
        self._climber.set_climber_motor_voltage(self._climber_pid)

    def end(self):


