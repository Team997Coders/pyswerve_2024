import logging
import robotpy_apriltag
import photonlibpy
import photonlibpy.photonCamera
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import *

import swerve
from config import PhotonCameraConfig
from swerve import SwerveDrive
import wpimath.geometry as geom
import time


class PhotonVisionAprilTagDetector:
    photonvision: PhotonCamera
    apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    swerve_drive: SwerveDrive 
    cam_position: geom.Transform3d
    photon_pose_estimatior: PhotonPoseEstimator
    logger: logging.Logger
    last_print: float | None # The last time we printed a log message
    apriltag_seen: bool

    def __init__(self, swerve_drive: swerve.SwerveDrive,
                       config: PhotonCameraConfig,
                       logger: logging.Logger):
        self.logger = logger
        self.last_print = None
        self.cam_position = config.camera_position
        self.apriltagfieldlayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2024Crescendo)
        self.apriltag_seen = False
        self.swerve_drive = swerve_drive
        self.photonvision = PhotonCamera(config.camera_name)
        self.photon_pose_estimatior = PhotonPoseEstimator(self.apriltagfieldlayout,
                                                          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                                          self.photonvision, self.cam_position)

    @property
    def best_target(self):
        results = self.photonvision.getLatestResult()
        targets = results.multiTagResult
        if targets:
            return results.targets[0]
        
        return 
    
    def periodic(self):
        #target = self.best_target
        if self.photonvision.isConnected():
            result = self.photonvision.getLatestResult()
            if result is not None:

                pose = self.photon_pose_estimatior.update(result)
                if pose is None: 
                    return

                if not self.apriltag_seen:
                    self.apriltag_seen = True
                    self.swerve_drive.pose = pose.estimatedPose.toPose2d()
                else:
                    self.swerve_drive.odemetry.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                                                    pose.timestampSeconds)
                
                if self.last_print is None or time.monotonic() - self.last_print > 10:
                #     self.logger.info(f"April tag {result.getTargets()} at {result.getTimestamp()}")
                    self.last_print = time.monotonic()
                    self.logger.info(f"Pose estimate: {pose}")

            else:
                self.apriltag_seen = False