import commands2
from subsystems.indexer import Indexer


class IndexOnIntake(commands2.InstantCommand):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._indexer = indexer

    def execute(self):
        self._indexer.speed = self._indexer.config.intake_velocity
        # self._indexer.voltage = 5
        print("IndexOn")


class IndexOnShoot(commands2.InstantCommand):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._indexer = indexer

    def execute(self):
        self._indexer.speed = self._indexer.config.shoot_velocity
        # self._indexer.voltage = 5
        print("IndexOn")

class IndexOff(commands2.InstantCommand):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        super().__init__()
        self._indexer = indexer

    def execute(self):
        self._indexer._indexer_motor.stopMotor()
        print("IndexOff")
