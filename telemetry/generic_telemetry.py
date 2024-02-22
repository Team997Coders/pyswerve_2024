
from typing import Callable
from config import PIDConfig
from wpilib import SmartDashboard as sd
import commands2
import rev

class float_editbox(commands2.Command):
    _get_value: Callable[[], float]
    _set_value: Callable[[float], None]
    _last_known_value: float
    _name: str

    def __init__(self, name: str,
                       get_value: Callable[[], float],
                       set_value: Callable[[float], None],
                       default_value: float | None = 0.0):
        self._name = name
        self._set_value = set_value
        self._get_value = get_value

        if default_value is not None:
            sd.setDefaultNumber(name, default_value)

        self._last_known_value = self._get_value()
        sd.putNumber(name, get_value())

    def execute(self):

        new_value = sd.getNumber(self._name, self._get_value())
        if new_value != self._last_known_value:
            self._last_known_value = new_value
            self._set_value(sd.getNumber(self._name, self._get_value()))
            print("Updating {self._name} to {new_value}")

        else:
            #No change, display the most recent value
            current_value = self._get_value()
            if current_value != self._last_known_value:
                self._last_known_value = current_value
                sd.putNumber(self._name, current_value)