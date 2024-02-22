
import navx
import commands2
from swerve import swervedrive
class reset_gyro(commands2.InstantCommand):
    pose : swervedrive.SwerveDrive
    def __init__(self):
        # Calls the constructor of our parent class if it exists.  Do this first when inheriting from a parent class.
        # This call is most likely required for a submodule to work properly
        super().__init__()
        self._pose = self.pose
    @property
    def navx(self) -> navx.AHRS:
        return self.navx
    def execute(self):
        self._pose.reset_pose()
        self.navx.reset()

