from typing import Optional
import time
import commands2
import wpilib
import wpimath.controller
import rev
import logging
import robot_config
from robot_config import shooter_config
from robot_config import intake_config
from subsystems import Intake, Indexer
from commands2 import Command


class IntakeOn(commands2.Command):
    _intake: Intake

    def __init__(self, intake):
        """Pass other subsystems and a logger to this subsystem for debugging

        :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
        :param logger: A python built-in package that handles writing logging messages to netconsole
        """
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._intake = intake

    def execute(self):
        self._intake.velocity = 1


class IntakeOff(commands2.Command):
    _intake: Intake

    def __init__(self, intake):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()
        self._intake = intake

    def execute(self):
        self._intake.velocity = 0


class Load(commands2.Command):
    _command: commands2.Command

    def __init__(self, intake: Intake, indexer: Indexer):
        super().__init__()
        self._command = commands2.cmd.sequence(
            IntakeOn(intake),
            commands2.cmd.waitUntil(lambda: indexer.ready),
            IntakeOff(intake)
        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
