import rev
from config import PIDConfig, MotorConfig, ProfiledPIDConfig
from wpimath.controller import ProfiledPIDControllerRadians, ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians


def init_pid(pid: rev.SparkMaxPIDController, pid_config: PIDConfig, feedback_device: rev.CANSensor | None = None):
    """Configures a SparkMax PID controller with the provided PIDConfig"""
    if feedback_device is not None:
        pid.setFeedbackDevice(feedback_device)
    pid.setP(pid_config.p)
    pid.setI(pid_config.i)
    pid.setD(pid_config.d)

    if pid_config.wrapping is not None and \
            (pid_config.wrapping.min is not None or pid_config.wrapping.max is not None):
        pid.setPositionPIDWrappingEnabled(True)
        if pid_config.wrapping.min is not None:
            pid.setPositionPIDWrappingMinInput(pid_config.wrapping.min)
        if pid_config.wrapping.max is not None:
            pid.setPositionPIDWrappingMaxInput(pid_config.wrapping.max)

    else:
        pid.setPositionPIDWrappingEnabled(False)


def init_motor(motor: rev.CANSparkMax, config: MotorConfig):
    motor.restoreFactoryDefaults()
    motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
    motor.setInverted(config.inverted)
    if config.open_ramp_rate is not None:
        motor.setOpenLoopRampRate(config.open_ramp_rate)
    if config.closed_ramp_rate is not None:
        motor.setOpenLoopRampRate(config.closed_ramp_rate)
    if config.current_limit is not None:
        motor.setOpenLoopRampRate(config.current_limit)


def create_profiled_pid_radians(pid_config: ProfiledPIDConfig) -> ProfiledPIDControllerRadians:
    pid = ProfiledPIDControllerRadians(
        Kp=pid_config.p,
        Ki=pid_config.i,
        Kd=pid_config.d,
        constraints=TrapezoidProfileRadians.Constraints(pid_config.profile.velocity, pid_config.profile.acceleration))

    if pid_config.wrapping is not None:
        pid.enableContinuousInput(minimumInput=pid_config.wrapping.min,
                                  maximumInput=pid_config.wrapping.max)
    if pid_config.tolerance is not None:
        pid.setTolerance(positionTolerance=pid_config.tolerance.position,
                         velocityTolerance=pid_config.tolerance.velocity)
    return pid


def create_profiled_pid(pid_config: ProfiledPIDConfig) -> ProfiledPIDController:
    pid = ProfiledPIDController(
        Kp=pid_config.p,
        Ki=pid_config.i,
        Kd=pid_config.d,
        constraints=TrapezoidProfileRadians.Constraints(pid_config.max_velocity, pid_config.max_acceleration))

    if pid_config.wrapping is not None:
        pid.enableContinuousInput(minimumInput=pid_config.wrapping.min,
                                  maximumInput=pid_config.wrapping.max)
    if pid_config.tolerance is not None:
        pid.setTolerance(positionTolerance=pid_config.tolerance.position,
                         velocityTolerance=pid_config.tolerance.velocity)
    return pid


def create_pid(pid_config: PIDConfig) -> ProfiledPIDControllerRadians:
    pid = PIDController(
        Kp=pid_config.p,
        Ki=pid_config.i,
        Kd=pid_config.d)

    if pid_config.tolerance is not None:
        pid.setTolerance(positionTolerance=pid_config.tolerance.position,
                         velocityTolerance=pid_config.tolerance.velocity)
