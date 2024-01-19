import abc
import wpimath
import wpimath.units
import wpimath.geometry as geom
import wpimath.kinematics as kinematics
from config import ModulePosition

class ISwerveModule(abc.ABC):
    @abc.abstractproperty
    def location(self) -> geom.Translation2d:
        '''Location of the module relative to robot center in meters'''
        raise NotImplementedError()

    @abc.abstractproperty
    def velocity(self) -> float:
        '''Velocity of the drive motor in meters per second'''
        raise NotImplementedError()

    @velocity.setter
    @abc.abstractmethod
    def velocity(self, meters_per_sec: float):
        '''Velocity of the drive motor in meters per second'''
        raise NotImplementedError()

    @abc.abstractproperty
    def angle(self) -> float:
        '''Rotation of the module in radians'''
        raise NotImplementedError()

    @angle.setter
    @abc.abstractmethod
    def angle(self, angle: float):
        '''Rotation of the module in radians'''
        raise NotImplementedError()

    @abc.abstractproperty
    def position(self) -> kinematics.SwerveModulePosition:
        raise NotImplementedError()

    @abc.abstractproperty
    def rotation2d(self) -> geom.Rotation2d:
        '''Current rotation of the module'''
        raise NotImplementedError()

    @abc.abstractproperty
    def desired_state(self) -> kinematics.SwerveModuleState:
        '''The desired state of the module'''
        raise NotImplementedError()

    @desired_state.setter
    @abc.abstractmethod
    def desired_state(self, value: kinematics.SwerveModuleState):
        '''The desired state of the module'''
        raise NotImplementedError()

    @abc.abstractproperty
    def measured_state(self) -> kinematics.SwerveModuleState:
        '''The measured state of the module based on sensor inputs'''
        raise NotImplementedError()

    @abc.abstractmethod
    def initialize(self) -> bool:
        '''Initialize the swerve module.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True.'''
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        '''Set voltage of all motors to zero'''
        raise NotImplementedError()


class ISwerveDrive(abc.ABC):
    @abc.abstractmethod
    def num_modules(self) -> int:
        '''Returns the number of swerve modules'''
        raise NotImplementedError()

    '''Interface for a swerve drive'''

    @abc.abstractmethod
    def drive(self, v_x: float, v_y: float, rotation: wpimath.units.radians_per_second):
        '''Drive the robot using cartesian coordinates'''
        raise NotImplementedError()

    @abc.abstractmethod
    def lock_wheels(self):
        '''Lock the wheels in place'''
        raise NotImplementedError()

    @abc.abstractproperty
    def modules(self) -> dict[ModulePosition, ISwerveModule]:
        '''Returns a dictionary of swerve modules'''
        raise NotImplementedError()

    @abc.abstractproperty
    def ordered_modules(self) -> list[ISwerveModule]:
        '''Returns a list of swerve modules in order.  This order should not change after initialization as it is passed to wpilib functions that depend on consistent ordering'''
        raise NotImplementedError()

    @abc.abstractmethod
    def initialize(self) -> bool:
        '''Initialize the swerve drive.  Can be called repeatedly until it returns True.
           If no initialization is needed simply implement this method to return True.'''
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        '''Set voltage of all motors to zero'''
        raise NotImplementedError()
