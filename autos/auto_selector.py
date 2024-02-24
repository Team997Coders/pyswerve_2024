import wpilib


def create_auto_selector(sd_path: str, autos : list[str]) -> wpilib.SendableChooser:
    """Creates a widget in smart dashboard that can select which test group to run from a list
    :param sd_path: The path in smart dashboard to write the selected test group to
    """

    chooser = wpilib.SendableChooser()
    for i, path in enumerate(autos):
        chooser.addOption(path, i)  # Write the index to the dashboard, we'll use this to lookup which test to run

    path = autos[-1]  # The last test in the test group is the default
    chooser.setDefaultOption(path, len(autos) - 1)
    wpilib.SmartDashboard.putData(sd_path, chooser)
    return chooser
