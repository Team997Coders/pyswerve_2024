# #Packages related to FIRST can be found at https://robotpy.readthedocs.io/en/stable/ as various API's
# #If you add a package check if it is listed in the pyproject.toml file to ensure it deploys to the robot
# from typing import Optional
#
# import commands2
# import wpilib
# import rev
# import logging
#
# from commands2 import Command
#
#
# class TemplateSubsystem(commands2.subsystem.Subsystem):
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
#     def __init__(self, command_scheduler: commands2.CommandScheduler, logger: logging.Logger):
#         """Pass other subsystems and a logger to this subsystem for debugging
#
#         :param command_scheduler: Defined in robot.py, allows registering the subsystem and schedules commands
#         :param logger: A python built-in package that handles writing logging messages to netconsole
#         """
#         # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
#         # This call is most likely required for a submodule to work properly
#         super().__init__()
#
#         # Save the command scheduler in case we need to make changes later
#         self._command_scheduler = command_scheduler
#
#         # Create a logger that prepends the name of the subsystem to output
#         self.logger = logger.getChild(type(
#             self).__name__)  # type(self).__name__ returns the name of the class.  Pass a string if you want something different
#
#         # Note, logging is slow, and not used in competition.  Turn it off
#         # logging when not debugging by doing this:
#         # if __debug__:
#         #     self.logger.info("My log message that doesn't need to print in competition")
#
#         self._command_scheduler.registerSubsystem(self)
#
#     def getName(self) -> str:
#         return type(self).__name__ # Replace with a string if you want a friendlier name
#
#     def periodic(self) -> None:
#         super().periodic()
#
#     def getCurrentCommand(self) -> Optional[Command]:
#         return super().getCurrentCommand()
#
#     def getDefaultCommand(self) -> Optional[Command]:
#         return super().getDefaultCommand()
#
#
#
