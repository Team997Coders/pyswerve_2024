import limelight
import limelightresults
import robotpy_apriltag
from swerve import SwerveDrive
from wpimath import geometry as geom
import logging
import threading
import time
from swerve import SwerveDrive
import wpimath.geometry as geom
from config import CameraConfig

class LimelightAprilTagDetector:

    apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    swerve_drive: SwerveDrive
    cam_position: geom.Transform3d
    logger: logging.Logger
    last_print: float | None # The last time we printed a log message
    apriltag_seen: bool
    limelight: limelight.Limelight | None = None
    limelight_thread: threading.Thread | None = None

    def __init__(self, config: CameraConfig, logger: logging.Logger):
        self.logger = logger
        self.cam_position = config.camera_position
        discovered_limelights = limelight.discover_limelights()
        print("discovered limelights:", self.discovered_limelights)

        if discovered_limelights:
            limelight_address = discovered_limelights[0]
            self.limelight = limelight.Limelight(limelight_address)
            results = limelight.get_results()
            print("targeting results:", results)

            limelight.enable_websocket()

            self.limelight_thread = threading.Thread(target=self.periodic).start()


    def periodic(self):
        while True:
            result = limelight.get_latest_results()

            parsed_result = limelightresults.parse_results(result)
            if parsed_result is not None:
                #print(parsed_result.pipeline_id)
                #print(parsed_result.parse_latency)

                # Accessing arrays
                for tag in parsed_result.fiducialResults:
                    print(tag.robot_pose_target_space, tag.fiducial_id)
                    self.swerve_drive.add_vision_measurement(
                        tag.robot_pose_target_space, result.timestamp)

            time.sleep(0.5)