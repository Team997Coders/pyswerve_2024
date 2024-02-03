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
from swerve import SwerveDrive
from debug import attach_debugger
import wpimath.kinematics as kinematics
from wpilib import SmartDashboard, Field2d
from wpilib import SmartDashboard as sd
import commands2
from math_helper import Range
from math_help import processControllerDeadband


if __debug__ and "run" in sys.argv:
    #To enter debug mode, add the --debug flag to the deploy command:
    #python -m robotpy deploy --debug
    #At the time this was written, you have to wait for the robot code to start before attempted to attach the debugger.
    attach_debugger() 

class MyRobot(commands2.TimedCommandRobot):

    # _command_scheduler: commands2.CommandScheduler

    swerve_drive: SwerveDrive
    swerve_telemetry: telemetry.SwerveTelemetry
    test_driver: TestDriver
    _navx: navx.AHRS  # Attitude Heading Reference System

    controller: wpilib.XboxController
    joyStick: wpilib.Joystick
 
    photonvision: ntcore.NetworkTable | None

    field: wpilib.Field2d
 
    @property
    def navx(self) -> navx.AHRS:
        return self._navx

    def robotInit(self):
        super().robotInit()
        # self._command_scheduler = commands2.CommandScheduler()
        self._navx = navx.AHRS.create_spi()
        self.controller = wpilib.XboxController(0)
        self.joyStick = wpilib.Joystick(0)
        self.field = wpilib.Field2d()
        self.swerve_drive = swerve.SwerveDrive(self._navx, robot_config.swerve_modules, robot_config.physical_properties, self.logger)
        self.swerve_telemetry = telemetry.SwerveTelemetry(self.swerve_drive, robot_config.physical_properties)
        SmartDashboard.putData("Field", self.field)
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
        self.field.setRobotPose(self.swerve_drive.pose)
        sd.putData("Field", self.field)
            

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
        self.doGamepadInput()
    
    def doGamepadInput(self):
        vx = self.controller.getRawAxis(1)
        vy = self.controller.getRawAxis(0)

        theta = self.controller.getRawAxis(4)

        vx_adjusted = processControllerDeadband(vx, Range(robot_config.teleop_controls.x_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed))
        vy_adjusted = processControllerDeadband(vy, Range(robot_config.teleop_controls.y_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed))
        theta_adjusted = processControllerDeadband(theta, Range(robot_config.teleop_controls.theta_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed))
        
        requested_speed = math.sqrt(vx_adjusted ** 2 + vy_adjusted ** 2) 

        # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
        if requested_speed > robot_config.physical_properties.max_drive_speed:
            scale_factor = robot_config.physical_properties.max_drive_speed / requested_speed
            vx_adjusted *= scale_factor
            vy_adjusted *= scale_factor

        SmartDashboard.putNumberArray("outputs", [vx_adjusted, vy_adjusted, theta_adjusted])
        self.sendDriveCommand(vx_adjusted, vy_adjusted, theta_adjusted)

    def doGameStickInput(self):
        vx = self.controller.getRawAxis(0)
        vy = self.controller.getRawAxis(1)

        theta = self.controller.getRawAxis(2)

        vx_adjusted = processControllerDeadband(vx, Range(robot_config.teleop_controls.x_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed))
        vy_adjusted = processControllerDeadband(vy, Range(robot_config.teleop_controls.x_deadband, 1), Range(0, robot_config.physical_properties.max_drive_speed))
        theta_adjusted = processControllerDeadband(theta, Range(robot_config.teleop_controls.theta_deadband, 1), Range(0, robot_config.physical_properties.max_rotation_speed))
        
        requested_speed = math.sqrt(vx_adjusted ** 2 + vy_adjusted ** 2) 

        # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
        if requested_speed > robot_config.physical_properties.max_drive_speed:
            scale_factor = robot_config.physical_properties.max_drive_speed / requested_speed
            vx_adjusted *= scale_factor
            vy_adjusted *= scale_factor



        SmartDashboard.putNumberArray("outputs", [vx_adjusted, vy_adjusted, theta_adjusted])

        # requested_speed = math.sqrt(vx * vx + vy * vy) 

        # Scale the vector vx, vy so that the magnitude of the vector does not exceed robot_config.physical_properties.max_drive_speed
        # if requested_speed > robot_config.physical_properties.max_drive_speed:
        #     scale_factor = robot_config.physical_properties.max_drive_speed / requested_speed
        #     vx *= scale_factor
        #     vy *= scale_factor

        self.sendDriveCommand(vx_adjusted,  vy_adjusted, theta_adjusted)

    def sendDriveCommand(self, vx: float, vy: float, theta: float):

        #velocity = math.sqrt(self.swerve_drive.measured_chassis_speed.vx ** 2 + self.swerve_drive.measured_chassis_speed.vy ** 2)
        
        if False: #velocity < 0.01 and theta == 0 and vx == 0 and vx == 0: # and velocity < .5 and self.swerve_drive.chassis_speed.omega < .5
            # TODO: Make sure robot is not actually moving too
            self.swerve_drive.lock_wheels()
            # pass
        else:
            self.swerve_drive.drive(-vx, -vy, -theta, None)

    def updateField(self):
        pass

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
