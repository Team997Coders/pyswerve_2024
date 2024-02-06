import abc
import rev
import wpimath.units
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
from typing import Sequence
from config import ModulePosition


class ISwerveModule(abc.ABC):

    @property
    @abc.abstractmethod
    def id(self) -> ModulePosition:
        """Which module this is, used for ordering and naming"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def location(self) -> geom.Translation2d:
        """Location of the module relative to robot center in meters"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def velocity(self) -> float:
        """Velocity of the drive motor in meters per second"""
        raise NotImplementedError()

    @velocity.setter
    @abc.abstractmethod
    def velocity(self, meters_per_sec: float):
        """Velocity of the drive motor in meters per second"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def angle(self) -> float:
        """Rotation of the module in radians"""
        raise NotImplementedError()

    @angle.setter
    @abc.abstractmethod
    def angle(self, angle: float):
        """Rotation of the module in radians"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def position(self) -> kinematics.SwerveModulePosition:
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def rotation2d(self) -> geom.Rotation2d:
        """Current rotation of the module"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def desired_state(self) -> kinematics.SwerveModuleState:
        """The desired state of the module"""
        raise NotImplementedError()

    @desired_state.setter
    @abc.abstractmethod
    def desired_state(self, value: kinematics.SwerveModuleState):
        """The desired state of the module"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def measured_state(self) -> kinematics.SwerveModuleState:
        """The measured state of the module based on sensor inputs"""
        raise NotImplementedError()

    @abc.abstractmethod
    def initialize(self) -> bool:
        """Initialize the swerve module.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True."""
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """Set voltage of all motors to zero"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def drive_motor(self) -> rev.CANSparkMax:
        """The drive motor for the module"""
        raise NotImplementedError()

    @property
    @abc.abstractmethod
    def angle_motor(self) -> rev.CANSparkMax:
        """The drive motor for the module"""
        raise NotImplementedError()

    @abc.abstractmethod
    def rotate_drive_wheel(self, num_rotations):
        """The drive motor for the module"""
        raise NotImplementedError()

    @abc.abstractmethod
    def drive_set_distance(self, meters: float, angle: float):
        """Drive the wheel a specific distance in meters.  If you use this call, desired state speed will be incorrect until
           you set desired_state again. This is because the PID controller will be driving the wheel to the specified distance."""
        raise NotImplementedError()


class ISwerveDrive(abc.ABC):
    @abc.abstractmethod
    def num_modules(self) -> int:
        """Returns the number of swerve modules"""
        raise NotImplementedError()

    '''Interface for a swerve drive'''

    @abc.abstractmethod
    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second, run_modules: Sequence[ModulePosition] | None):
        """Drive the robot using cartesian coordinates

        :param run_modules: A set of modules to drive.  If None, all modules will be driven.  This is useful for testing individual modules and ensuring ModulePosition is correct for each module
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def lock_wheels(self):
        """Lock the wheels in place"""
        raise NotImplementedError()

    @abc.abstractproperty
    def modules(self) -> dict[ModulePosition, ISwerveModule]:
        """Returns a dictionary of swerve modules"""
        raise NotImplementedError()

    @abc.abstractproperty
    def ordered_modules(self) -> list[ISwerveModule]:
        """Returns a list of swerve modules in order.  This order should not change after initialization as it is passed to wpilib functions that depend on consistent ordering"""
        raise NotImplementedError()

    @abc.abstractmethod
    def initialize(self) -> bool:
        """Initialize the swerve drive.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True."""
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """Set voltage of all motors to zero"""
        raise NotImplementedError()
    
    @abc.abstractproperty
    def odemetry(self) -> kinematics.SwerveDrive4Odometry:
        raise NotImplementedError()
    
    @abc.abstractmethod
    def periodic(self):
        raise NotImplementedError()