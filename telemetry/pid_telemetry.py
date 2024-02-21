
from typing import Callable
from config import PIDConfig
from wpilib import SmartDashboard as sd
import commands2


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

class PIDEditor(commands2.Command):

    def __init__(self, name: str, get_pid: Callable[[], PIDConfig], set_pid: Callable[[PIDConfig], None]):
        self._name = name
        self._set_pid = set_pid
        self._pid = get_pid

        self._p = float_editbox(f"{name}_p", lambda: get_pid().p, lambda x: self._set_pid(PIDConfig(x, get_pid().i, get_pid().d)))
        self._i = float_editbox(f"{name}_i", lambda: get_pid().i, lambda x: self._set_pid(PIDConfig(get_pid().p, x, get_pid().d)))
        self._d = float_editbox(f"{name}_d", lambda: get_pid().d, lambda x: self._set_pid(PIDConfig(get_pid().p, get_pid().i, x)))

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(
            commands2.SequentialCommandGroup(
                commands2.ParallelCommandGroup(self._p, self._i, self._d),
                commands2.WaitCommand(1)
            )
        )
