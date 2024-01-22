'''This package contains read-only record objects used for configuration'''

from .range_config import Range, OptionalRange
from .pid import PIDConfig

from . import physical_config
from .physical_config import PhysicalConfig

from . import swerve_module_config
from .swerve_module_config import SwerveModuleConfig, MotorConfig, \
                                  EncoderConfig, SwerveModuleFloatProperty, \
                                  ModulePosition, SwerveModuleIntProperty, \
                                  OptionalSwerveModuleFloatProperty, OptionalSwerveModuleIntProperty

from . import driver_controls
from .driver_controls import DriverControlsConfig