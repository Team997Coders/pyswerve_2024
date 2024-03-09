import commands2
import commands
from typing import Sequence

def shoot_drive_load_backup_auto(robot) -> Sequence[commands2.Command]:

    if robot.shooter is None or robot.indexer is None or robot.intake is None:
        raise ValueError("Robot must have a shooter, indexer, and intake to run this auto")

    cmd = commands2.cmd.ParallelRaceGroup(
        commands.GotoXYTheta(robot.swerve_drive, (2, 0, 0),
                             robot._x_axis_control, robot._y_axis_control, robot._heading_control))
    return cmd