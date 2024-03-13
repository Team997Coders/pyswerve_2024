"""Settings that apply to all robots should live here"""

import math_help
from config import OptionalSwerveModuleFloatProperty, SwerveModuleIntProperty, RevConfig

swerve_current_limit = SwerveModuleIntProperty(drive=40, angle=40)
swerve_ramp_rate = OptionalSwerveModuleFloatProperty(drive=0.25, angle=0.25)

rev_config = RevConfig()
