from typing import NamedTuple

class DriverControlsConfig(NamedTuple):
    x_deadband: float
    y_deadband: float
    theta_deadband: float
