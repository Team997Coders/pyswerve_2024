# import auto_selector
from . import auto_calibrations
from . import manual_autos
from typing import NamedTuple, Callable, Sequence, Any
import commands2

class AutoFactory(NamedTuple):
    """Stores information to create an autonomous mode command(s).  The resulting commands must be added to the scheduler in AutonomousInit"""
    name: str
    create: Callable[[Any], commands2.Command | Sequence[commands2.Command]]
    args: Sequence[Any]