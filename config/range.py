from typing import NamedTuple

class Range(NamedTuple):
    """Describes a range of values"""
    min: float
    max: float

class OptionalRange(NamedTuple):
    """Describes a range of values.  Either min or max can be None"""
    min: float | None
    max: float | None