import time

import commands2
import swerve
import telemetry.limelight_nettables
from config import LimelightCameraConfig
import logging
from swerve import SwerveDrive
import wpimath.geometry as geom
import threading


class LimeLightPositioning(commands2.Subsystem):
    """This class launches a thread upon creation and periodically updates the robot pose based on the limelight camera"""

    _limelight_tables: telemetry.LimeLightNetTables
    _swerve_drive: SwerveDrive
    _logger: logging.Logger
    _last_print: float | None  # The last time we printed a log message
    _apriltag_seen: bool
    _thread: threading.Thread | None = None  #Thread used to update pose in the background
    _update_delay: float # Amount of time to wait between updates
    _config: LimelightCameraConfig

    @property
    def cam_position(self) -> geom.Transform3d:
        return self._config.camera_position

    def __init__(self, swerve_drive: swerve.SwerveDrive,
                       config: LimelightCameraConfig,
                       logger: logging.Logger,
                       use_thread: bool = False):
        self._logger = logger
        self._last_print = None
        self._apriltag_seen = False
        self._swerve_drive = swerve_drive
        self._limelight_tables = telemetry.LimeLightNetTables(config.camera_name)
        self._config = config
        self._update_delay = 1.0 / config.refresh_rate
        self._thread = None
        self._use_thread = use_thread

    def start_thread(self):
        self._logger.info("Starting Limelight Pose Update Thread")
        self._thread = threading.Thread(target=self.run_thread())
        self._thread.daemon = True  # Do not allow the pose update thread to die if a loop overrun kills the loop thread that created us
        self._thread.start()

    def periodic(self):
        if not self._use_thread:
            self.update_pose()
        elif self._thread is None or not self._thread.is_alive():
             self.start_thread()

    def run_thread(self):
        while True:
            self.update_pose()
            time.sleep(self._update_delay)

    def update_pose(self):
        pose, timestamp = self._limelight_tables.getBotPoseEstimate()
        if pose is None:
            return

        #Camera should be measured relative to the center of the robot
        #So if the camera is centered on the ground directly front of the robot center by 10cm,
        # that is recorded as +10cm on the X axis.  We need to adjust the pose back by 10cm
        camera_adjusted_pose = pose.transformBy(self.cam_position.inverse())
        #camera_adjusted_pose = pose
        if not self._apriltag_seen:
            self._apriltag_seen = True
            self._swerve_drive.reset_pose(camera_adjusted_pose.toPose2d())
            #print(f"reset pose: {camera_adjusted_pose.toPose2d()}")
        else:
            self._swerve_drive.add_vision_measurement(camera_adjusted_pose.toPose2d(), timestamp)
            #print(f"add vision measurement: {camera_adjusted_pose.toPose2d()} @ {timestamp} estimated: {self._swerve_drive.estimated_position}")





