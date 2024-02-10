from typing import NamedTuple

import wpilib
import rev

class IndexerConfig(NamedTuple):
    feeder_motor_id: int
    intake_motor_id: int
    feeder_sensor_channel: int
    is_intake_motor_inverted: bool
    is_feeder_motor_inverted: bool



