import logging
import robotpy_apriltag
import photonlibpy
import photonlibpy.photonCamera
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import *
from swerve import SwerveDrive
import wpimath.geometry as geom
import time


class AprilTagDetector:

    photonvision: PhotonCamera
    apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    swerve_drive: SwerveDrive 
    cam_position: geom.Transform3d
    photon_pose_estimatior: PhotonPoseEstimator
    logger: logging.Logger

    def __init__(self, swerve_drive, logger: logging.Logger):
        self.logger = logger
        self.photonvision = PhotonCamera("Camera3")
        self.apriltagfieldlayout = robotpy_apriltag.loadAprilTagLayoutField(robotpy_apriltag.AprilTagField.k2024Crescendo)
        self.swerve_drive = swerve_drive
        translation = geom.Translation3d((0.305) , 0, (0.152)) # inches(12, 0, 6)
        rotation = geom.Rotation3d(0, 0, 0)
        self.cam_position = geom.Transform3d(translation, rotation)
        self.photon_pose_estimatior = PhotonPoseEstimator(self.apriltagfieldlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.photonvision, self.cam_position)
        self.last_print = time.monotonic()
    
    @property
    def best_target(self):
        results = self.photonvision.getLatestResult()
        targets = results.multiTagResult
        if targets:
            return results.targets[0]
        
        return 
    
    def periodic(self):
        #target = self.best_target
        result = self.photonvision.getLatestResult()
        if result is not None:

            pose = self.photon_pose_estimatior.update(result)
            if pose is None: 
               return 
            
            if time.monotonic() - self.last_print > 10:
            #     self.logger.info(f"April tag {result.getTargets()} at {result.getTimestamp()}")
                self.last_print = time.monotonic()
                self.logger.info(f"Pose estimate: {pose}")
                
            self.swerve_drive.odemetry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds)   