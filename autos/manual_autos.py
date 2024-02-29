import commands2
import commands

def shoot_drive_load_backup_auto(robot):
    cmd = commands2.cmd.ParallelRaceGroup(
            commands2.cmd.sequence(
                commands.Shoot(robot.shooter, robot.indexer),
                commands2.cmd.WaitCommand(
                    robot.config.shooter_config.default_spinup_delay + robot.config.shooter_config.default_fire_time),
                commands2.cmd.ParallelCommandGroup(
                    commands.Load(robot.intake, robot.indexer),
                    commands2.cmd.sequence(
                        commands.GotoXYTheta(robot.swerve_drive, (2, 0, 0),
                                             robot._x_axis_control, robot._y_axis_control, robot._heading_control),
                        commands.GotoXYTheta(robot.swerve_drive, (-2, 0, 0),
                                             robot._x_axis_control, robot._y_axis_control, robot._heading_control),
                    ),
                ),
            ),
            commands2.cmd.WaitCommand(15)
        )
    return cmd