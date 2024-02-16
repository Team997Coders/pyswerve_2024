import commands2
from subsystems.indexer import Indexer


class IndexOn(commands2.InstantCommand):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

        self._indexer = indexer

    def execute(self):
        self._indexer.velocity = self._indexer.config.default_velocity


class IndexOff(commands2.InstantCommand):
    _indexer: Indexer

    def __init__(self, indexer: Indexer):
        super().__init__()
        self._indexer = indexer

    def execute(self):
        self._indexer.velocity = 0
