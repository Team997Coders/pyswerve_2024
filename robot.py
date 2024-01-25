import sys 
import wpilib
import rev 
import config 
import robot_config
import swerve
import time
import ntcore
import telemetry
import math
import navx
from drivers import TestDriver
from swerve import ISwerveDrive, ISwerveModule
from debug import attach_debugger
import wpimath.kinematics as kinematics

if __debug__ and "run" in sys.argv:
    #To enter debug mode, add the --debug flag to the deploy command:
    #python -m robotpy deploy --debug
    #At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger() 

class MyRobot(wpilib.TimedRobot):

    swerve_drive: ISwerveDrive
    swerve_telemetry: telemetry.SwerveTelemetry
    test_driver: TestDriver
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: wpilib.XboxController

    photonvision: ntcore.NetworkTable | None
 

    @property
    def navx(self) -> navx.AHRS:
        return self._navx

    def robotInit(self):
        super().robotInit()
        self._navx = navx.AHRS.create_spi()
        self.controller = wpilib.XboxController(0)
        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules, robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)

        self.swerve_drive.initialize()

        try:
            self.photonvision =  ntcore.NetworkTableInstance.getDefault().getTable("photonvision/Camera_Module_v2")
            if self.photonvision is not None:
                self.logger.info(f"Photonvision connected!")
            else:
                self.logger.error(f"Could not connect to PhotonVision.")
        except Exception as e:
            self.logger.error(f"Could not connect to PhotonVision.\n{e}")
            self.photonvision = None

        self.test_driver = TestDriver(self.swerve_drive, self.logger)
    

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.swerve_telemetry.report_to_dashboard()
        self.swerve_drive.periodic()
        self.update_position()
            

    def update_position(self) -> bool:
        if self.photonvision is None:
            return False
        
        has_qr_code = self.photonvision.getEntry("hasTarget").getBoolean(False)

        if(has_qr_code):
            self.logger.info(f"PhotonVision has_qr_code: {has_qr_code}")
            #TODO: Update pose position using angle to april tag and distance
            #self.swerve_drive.odemetry.resetPosition(kinematics.SwerveModulePosition(0,0,0), geom.Rotation2d(0))
            return True
        
        return False
  
    def teleopPeriodic(self):
        super().teleopPeriodic()

        vx = self.controller.getRawAxis(1)
        vy = self.controller.getRawAxis(0)

        theta = self.controller.getRawAxis(4)

        x_deadband = robot_config.teleop_controls.x_deadband
        y_deadband = robot_config.teleop_controls.y_deadband

        theta_deadband = robot_config.teleop_controls.theta_deadband

        if abs(theta) < theta_deadband:
            theta = 0

        if abs(vx) < x_deadband and abs(vy) < y_deadband and abs(theta) < theta_deadband:
            # TODO: Make sure robot is not actually moving too
            self.swerve_drive.lock_wheels()
        else:
            vx *= robot_config.physical_properties.max_drive_speed
            vy *= robot_config.physical_properties.max_drive_speed
            scaled_speed = math.sqrt(vx * vx + vy * vy) 

            # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
            if scaled_speed > robot_config.physical_properties.max_drive_speed:
                scale_factor = robot_config.physical_properties.max_drive_speed / scaled_speed
                vx *= scale_factor
                vy *= scale_factor

            self.swerve_drive.drive(-vx, -vy, theta * robot_config.physical_properties.max_rotation_speed)

    def autonomousInit(self):
        super().autonomousInit()

    def autonomousPeriodic(self):
        super().autonomousPeriodic()

    def testInit(self) -> None:
        super().testInit()  
        self.test_driver.testInit()
 
    def testPeriodic(self) -> None:
        super().testPeriodic()
        self.test_driver.testPeriodic()
