# import math
#
# import rev
# import wpilib
# import commands2
# from typing import Callable
#
# import hardware
# import logging
# from config import BarConfig
#
#
# class Bar(commands2.Subsystem):
#     _bar_motor: rev.CANSparkMax
#     _bar_encoder: rev.SparkRelativeEncoder
#     _bar_pid: rev.SparkMaxPIDController
#     config: BarConfig
#     _logger: logging.Logger
#
#
#     def __init__(self, config: BarConfig, logger: logging.Logger):
#         super().__init__()
#         self._logger = logger.getChild("bar")
#         self.config = config
#         self._bar_motor = rev.CANSparkMax(config.bar_motor.id, rev.CANSparkMax.MotorType.kBrushless)
#         hardware.init_motor(self._bar_motor, config.bar_motor)
#         self._bar_encoder = self._bar_motor.getEncoder()
#         self._bar_encoder.setPositionConversionFactor(2 * math.pi)
#         self._bar_pid = self._bar_motor.getPIDController()
#         hardware.init_pid(self._bar_pid, self.config.bar_pid)
#         self._bar_absolute_encoder = rev.SparkMaxAbsoluteEncoder
#         self._bar_absolute_encoder.setPositionConversionFactor()
#         self._bar_encoder.setPosition(0)
#
#
#     @property
#     def ready(self) -> bool:
#         return self._read_bar_state()
#
#     @property
#     def voltage(self):
#         return self._bar_motor.getBusVoltage()
#
#     @voltage.setter
#     def voltage(self, value: float):
#         self._bar_motor.setVoltage(value)
#
#     @property
#     def velocity(self) -> float:
#         return self._bar_encoder.getVelocity()
#
#     @velocity.setter
#     def velocity(self, value):
#         if value == 0:
#             self._bar_encoder.setPosition(0)
#         self._bar_pid.setReference(value, rev.CANSparkMax.ControlType.kVelocity)
#
#     # @property
#     # def position(self):
#
#
#
