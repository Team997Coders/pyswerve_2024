import rev
from config import PIDConfig, MotorConfig


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
