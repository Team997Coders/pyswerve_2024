# Packages related to FIRST can be found at https://robotpy.readthedocs.io/en/stable/ as various API's
# If you add a package check if it is listed in the pyproject.toml file to ensure it deploys to the robot
from typing import Optional
import commands2
import commands2.cmd
import subsystems
from commands import index


# Shooting Sequence
# 1a. Spin up motor
# 1b. Turn off indexer
# 2. Wait
# 3. Turn on indexer
# 4a. Spin down motor
# 4b. Turn off indexer


class SpinupShooter(commands2.InstantCommand):
    _shooter: subsystems.Shooter
    def __init__(self, shooter: subsystems.Shooter):
        super().__init__()
        self._shooter = shooter

    def execute(self):
        self._shooter.speed = self._shooter.config.default_velocity

class SpindownShooter(commands2.InstantCommand):
    _shooter: subsystems.Shooter

    def __init__(self, shooter: subsystems.Shooter):
        super().__init__()
        self._shooter = shooter

    def execute(self):
        self._shooter.speed = 0
        # print("SpinDownShooter")

class Shoot(commands2.InstantCommand):
    _command: commands2.Command

    def __init__(self, shooter: subsystems.Shooter, indexer: subsystems.Indexer):
        super().__init__()
        self._command = commands2.cmd.sequence(
            commands2.cmd.ParallelCommandGroup(
                SpinupShooter(shooter),
                index.IndexOff(indexer)
            ),
            commands2.WaitCommand(shooter.config.default_spinup_delay),
            index.IndexOnShoot(indexer),
            commands2.WaitCommand(shooter.config.default_fire_time),
            commands2.cmd.ParallelCommandGroup(
                SpindownShooter(shooter),
                index.IndexOff(indexer)
            )
        )

    def execute(self):
        commands2.CommandScheduler.getInstance().schedule(self._command)
        # print("Shoot Execute")
