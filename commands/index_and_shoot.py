# #Packages related to FIRST can be found at https://robotpy.readthedocs.io/en/stable/ as various API's
# #If you add a package check if it is listed in the pyproject.toml file to ensure it deploys to the robot
# from typing import Optional
# import time
# import commands2
# import wpilib
# import wpimath
# import rev
# import logging
# import robot_config
# from robot_config import shooter_constants
# from robot_config import indexer_constants
# from commands2 import Command
# from subsystems.shooter import Shooter
# from subsystems.indexer import Indexer
#
#
#
# class index_and_shoot(commands2.Command):
#     """An example barebones subsystem that can be copied when creating a new subsystem
#        Documentation for the subsystem class is at:
#        https://robotpy.readthedocs.io/projects/commands-v2/en/stable/commands2/Subsystem.html#commands2.Subsystem
#        """
#
#     # Member variables may be pre-declared and initialized here.  If they are not declared here it is best
#     # practice to set them in the __init__ method.
#     logger: logging.Logger
#     _command_scheduler: commands2.CommandScheduler # An underscore in Python indicates the member is private
#
#     #Here are some examples of declaring a motor and a sensor from the rev package.
#     #motor1 : rev.CANSparkMax
#     #sensor1 : rev.CANSensor
#
#     _shooter: Shooter
#     _indexer: Indexer
#
#     _feeder_pid: wpimath.controller.ProfiledPIDControllerRadians
#
#     def __init__(self, indexer, shooter, feeder_pid, shooter_pid, intake_pid):
#         """Pass other subsystems and a logger to this subsystem for debugging
#
#         :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
#         :param logger: A python built-in package that handles writing logging messages to netconsole
#         """
#         # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
#         # This call is most likely required for a submodule to work properly
#         super().__init__()
#
#         self._shooter = shooter
#         self._indexer = indexer
#         self._feeder_pid = feeder_pid
#         self._intake_pid = intake_pid
#         self._shooter_pid = shooter_pid
#         # self.feederVoltage : float
#         # self.intakeVoltage : float
#         # self.shooterVoltage : float
#
#     def initialize(self):
#
#     def execute(self):
#         status = self._indexer.get_sensor_status()
#         if not status:
#             self._shooter.set_left_motor_voltage(0)
#             self._indexer.set_intake_voltage(self._intake_pid)  # fix PIDs later
#             self._indexer.set_feeder_voltage(self._feeder_pid)
#             status = self._indexer.get_sensor_status()
#             if status:
#                 self._shooter.set_left_motor_voltage(self._shooter_pid)
#                 self._indexer.set_feeder_voltage(0)
#                 self._indexer.set_intake_voltage(0)
#         if status:
#             self._shooter.set_left_motor_voltage(self._shooter_pid)
#             self._indexer.set_feeder_voltage(0)
#             self._indexer.set_intake_voltage(0)
#             status = self._indexer.get_sensor_status()
#             commands2.WaitCommand(3)
#             self._indexer.set_feeder_voltage(self._feeder_pid)
#             if not status:
#                 self._shooter.set_left_motor_voltage(0)
#                 self._indexer.set_intake_voltage(self._intake_pid)
#                 self._indexer.set_feeder_voltage(self._feeder_pid)
#
#         # self._shooter.set_left_motor_voltage(self._shooter_pid.calculate())  # fix PIDs later
#         # self._indexer.set_intake_voltage(self._intake_pid.calculate())
#         # self._indexer.set_feeder_voltage(self._feeder_pid)
#
#     def end(self):
#
#
#
#     def getName(self) -> str:
#         return type(self).__name__  # Replace with a string if you want a friendlier name
#
#     def periodic(self) -> None:
#         super().periodic()
#
#     def getCurrentCommand(self) -> Optional[Command]:
#         return super().getCurrentCommand()
#
#     def getDefaultCommand(self) -> Optional[Command]:
#         return super().getDefaultCommand()
#     def is_finished(self):
#         return False
#
#
