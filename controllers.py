from typing import NamedTuple
from config.driver_controls import ControllerKey
import wpilib


# A dictionary of controllers
controllers = dict[ControllerKey, wpilib.Joystick |
                                  wpilib.XboxController |
                                  wpilib.PS4Controller |
                                  wpilib.PS5Controller]

