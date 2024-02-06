from typing import NamedTuple
from . import OptionalRange

class PIDConfig(NamedTuple):
    p: float
    i: float
    d: float
    wrapping: OptionalRange | None # If range, is PID wrapping is enabled