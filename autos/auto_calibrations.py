import commands
import commands2
import swerve
import subsystems


def create_drive_forward_and_back_auto(swerve_drive: swerve.SwerveDrive,
                                       x_axis_control: subsystems.AxisPositionControl,
                                       y_axis_control: subsystems.AxisPositionControl,
                                       heading_control: subsystems.ChassisHeadingControl):

    starting_pos = swerve_drive.pose.x, swerve_drive.pose.y
    target = (starting_pos[0] + 6, starting_pos[1])

    print(f'Going between {starting_pos} and {target}')
    cmd = commands2.cmd.sequence(
        commands.GotoXYTheta(swerve_drive, (target[0], target[1], 0),
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, (starting_pos[0], starting_pos[1], 0),
                             x_axis_control, y_axis_control, heading_control))

    drv_cmd = commands.drive.Drive(
        swerve_drive,
        get_x=lambda: x_axis_control.desired_velocity,
        get_y=lambda: y_axis_control.desired_velocity,
        get_theta=lambda: heading_control.desired_velocity
    )
    cmd.requirements = {x_axis_control, y_axis_control, heading_control}
    drv_cmd.requirements = {swerve_drive}

    return drv_cmd, cmd


def drive_a_square_1(swerve_drive: swerve.SwerveDrive,
                     x_axis_control: subsystems.AxisPositionControl,
                     y_axis_control: subsystems.AxisPositionControl,
                     heading_control: subsystems.ChassisHeadingControl):
    starting_pos = swerve_drive.pose.x, swerve_drive.pose.y, 0
    target_one = (starting_pos[0] + 4, starting_pos[1], 0)
    target_two = (starting_pos[0] + 4, starting_pos[1] + 4, 0)
    target_three = (starting_pos[0], starting_pos[1] + 4, 0)

    cmd = commands2.cmd.sequence(
        commands.GotoXYTheta(swerve_drive, target_one,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, target_two,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, target_three,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, starting_pos,
                             x_axis_control, y_axis_control, heading_control)
    )

    drv_cmd = commands.drive.Drive(
        swerve_drive,
        get_x=lambda: x_axis_control.desired_velocity,
        get_y=lambda: y_axis_control.desired_velocity,
        get_theta=lambda: heading_control.desired_velocity
    )

    cmd.requirements = {x_axis_control, y_axis_control, heading_control}
    drv_cmd.requirements = {swerve_drive}

    return drv_cmd, cmd


def drive_a_square_2(swerve_drive: swerve.SwerveDrive,
                     x_axis_control: subsystems.AxisPositionControl,
                     y_axis_control: subsystems.AxisPositionControl,
                     heading_control: subsystems.ChassisHeadingControl):
    starting_pos = swerve_drive.pose.x, swerve_drive.pose.y, 0
    target_one = (starting_pos[0] + 4, starting_pos[1], 0)
    target_two = (starting_pos[0] + 4, starting_pos[1] + 4, 0)
    target_three = (starting_pos[0], starting_pos[1] + 4, 0)

    cmd = commands2.cmd.sequence(
        commands.GotoXYTheta(swerve_drive, target_one,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, target_two,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, target_three,
                             x_axis_control, y_axis_control, heading_control),
        commands.GotoXYTheta(swerve_drive, starting_pos,
                             x_axis_control, y_axis_control, heading_control)
    )

    drv_cmd = commands.drive.Drive(
        swerve_drive,
        get_x=lambda: x_axis_control.desired_velocity,
        get_y=lambda: y_axis_control.desired_velocity,
        get_theta=lambda: heading_control.desired_velocity
    )

    cmd.requirements = {x_axis_control, y_axis_control, heading_control}
    drv_cmd.requirements = {swerve_drive}

    return drv_cmd, cmd
