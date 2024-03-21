import commands2
from subsystems.climber import Climber


class ClimberUp(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):

        self._climber.set_brake_mode()  # set brake mode
        self._climber.speed = -0.5  # goes up


class ClimberDown(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):
        self._climber.speed = 0.5   # climb down
        self._climber.set_brake_mode()  # set brake mode
        # if self._climber.read_climber_state:  # if the sensor is hit
        #     self._climber.speed = 0  # climber stop



class ClimberStop(commands2.InstantCommand):
    _climber: Climber

    def __init__(self, climber: Climber):
        super().__init__()
        self._climber = climber

    def execute(self):
        self._climber.speed = 0
