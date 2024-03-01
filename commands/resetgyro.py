
import navx
import commands2


class ResetGyro(commands2.InstantCommand):

    def __init__(self):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()

    @property
    def navx(self) -> navx.AHRS:
        return self.navx
    def execute(self):
        self.navx.reset()

