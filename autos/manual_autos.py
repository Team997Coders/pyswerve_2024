import commands2
import commands


def shoot_drive_load_backup_auto(robot):

    if robot.shooter is None or robot.indexer is None or robot.intake is None:
        raise ValueError("Robot must have a shooter, indexer, and intake to run this auto")

    cmd = commands2.cmd.sequence(commands.Shoot(robot.shooter, robot.indexer), commands2.WaitCommand(5))
    return cmd
