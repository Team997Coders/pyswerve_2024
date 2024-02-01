import logging
import robotpy_apriltag
import photonlibpy
import photonlibpy.photonCamera
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import *
from swerve import SwerveDrive
import wpimath.geometry as geom

class AprilTagDetector:

    photonvision: PhotonCamera
    apriltagfieldlayout: robotpy_apriltag.AprilTagFieldLayout
    swerve_drive: SwerveDrive 
    cam_position: geom.Transform3d
    photon_pose_estimatior: PhotonPoseEstimator

    def __init__(self, swerve_drive, logger: logging.Logger):
        self.photonvision = PhotonCamera("photonvision/Camera_Module_v2")
        self.apriltagfieldlayout = robotpy_apriltag.AprilTagFieldLayout()
        self.swerve_drive = swerve_drive
        translation = geom.Translation3d(30.5, 0, 16.5)
        rotation = geom.Rotation3d(0, 0, 0)
        self.cam_position = geom.Transform3d(translation, rotation)
        self.photon_pose_estimatior = PhotonPoseEstimator(self.apriltagfieldlayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, self.photonvision, self.cam_position)

    
    @property
    def best_target(self):
        results = self.photonvision.getLatestResult()
        targets = results.getTargets()
        if targets:
            bestTarget = targets[0]
            return bestTarget
        
        return 
    
    def periodic(self):
        target = self.best_target
        if target is not None:
            #Update robot pose
            pose = self.photon_pose_estimatior.update()
            if pose is None:
                return
            
            self.swerve_drive.odemetry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds)