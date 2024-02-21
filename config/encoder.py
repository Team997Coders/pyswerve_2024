import math


class EncoderConfig:
    """Information to configure an encoder on the RoboRIO"""
    id: int | None  # Encoder ID on the RoboRIO, if there is one
    offset: float | None  # Offset in radians.  Subtract this number from the absolute encoder value to get 0 degrees relative to robot chassis. Set to None if offset is configured in Rev Hardware Client
    conversion_factor: float | None  # Conversion factor from encoder ticks to radians
    inverted: bool

    def __init__(self, id_val: int | None = None, offset: float | None = None, conversion_factor: float | None = None,
                 inverted: bool = False):
        self.id = id_val
        self.offset = offset
        self.conversion_factor = conversion_factor
        self.inverted = inverted

        if self.offset is not None:
            while self.offset <= 0:  # The absolute encoder cannot be negative, so add 2*pi until it is positive
                self.offset = self.offset + (math.pi * 2.0)
