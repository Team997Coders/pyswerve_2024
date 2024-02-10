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
from robot_config import indexer_config
from commands2 import Command
from subsystems.shooter import Shooter
from subsystems.indexer import Indexer


class IndexOn(commands2.Subsystem):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._indexer = indexer

    def execute(self):
        self._indexer.velocity = 1


class IndexerOff(commands2.Command):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        super().__init__()

    def execute(self):
        self._indexer.velocity = 0
