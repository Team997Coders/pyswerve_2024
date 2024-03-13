import rev
from typing import Any, Callable
import time
import config
import logging
import threading
import queue

from config import PIDConfig, MotorConfig, ProfiledPIDConfig
from wpimath.controller import ProfiledPIDControllerRadians, ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from typing import NamedTuple
from robots.common import rev_config

rev_hardware_set_queue = queue.Queue()
rev_thread = None


class rev_hardware_call(NamedTuple):
    func: Callable[..., rev.REVLibError]
    args: Any
    kwargs: Any


def safe_set_rev(func: Callable[..., rev.REVLibError], *args, **kwargs):
    """
     Call the provided function to set hardware settings, retrying after a short delay if an error occurs.
     To use it, pass the name of the function and the arguments to this function.  Note, this can kill the
     periodic loop if values are updated during operation, so use with caution.  Consider using safe_set_rev_in_thread
     when in the periodic loop

     Example before: self.drive_motor_encoder.setPositionConversionFactor(1.0 / physical_config.gear_ratio.drive)
     Example After : self.safe_set(drive_motor_encoder.setPositionConversionFactor, 1.0 / physical_config.gear_ratio.drive)
    """

    nRetries = rev_config.fw_set_retries
    error = None
    for i in range(0, nRetries):
        error = func(*args, **kwargs)
        if error == rev.REVLibError.kOk:
            return
        time.sleep(rev_config.fw_set_retry_delay_sec)

    if error:
        logging.error(f"Error setting {func.__name__}: {error}")


def __safe_set_rev_run(worklist: queue.Queue):
    """Function to run in a daemon thread to set hardware settings.  It will run until the queue is empty."""
    time_to_wait = 1  # Time to wait before giving up and ending this thread
    last_work_item_time = time.monotonic()

    while True:
        try:
            call = worklist.get()
            last_work_item_time = time.monotonic()
            safe_set_rev(call.func, *call.args, **call.kwargs)
        except queue.Empty:
            if time.monotonic() - last_work_item_time > time_to_wait:
                # End our thread
                global rev_thread
                rev_thread = None
                return

            # Wait a bit before checking the queue again
            time.sleep(0.01)


def safe_set_rev_in_thread(func: Callable[..., rev.REVLibError], *args, **kwargs):
    """
        Call the provided function to set hardware settings, retrying after a short delay if an error occurs.
        To use it, pass the name of the function and the arguments to this function.  It creates a daemon
        thread that will attempt to call the function and retry if it fails.  The daemon function prevents
        the set operation from killing a periodic loop if values are updated during operation, at the cost
        of some ambiguity of when the firmware value is actually set.

        Example before: self.drive_motor_encoder.setPositionConversionFactor(1.0 / physical_config.gear_ratio.drive)
        Example After : self.safe_set(drive_motor_encoder.setPositionConversionFactor, 1.0 / physical_config.gear_ratio.drive)
        """

    global rev_thread, rev_hardware_set_queue

    rev_hardware_set_queue.put(rev_hardware_call(func, args, kwargs))  # Add the call to the queue of work to be done

    # Start a thread to work through the queue if one is not already running
    if rev_thread is None:
        rev_thread = threading.Thread(target=__safe_set_rev_run, args=(rev_hardware_set_queue,), daemon=True)
        rev_thread.start()


def init_pid(pid: rev.SparkMaxPIDController, pid_config: PIDConfig, feedback_device: rev.CANSensor | None = None):
    """Configures a SparkMax PID controller with the provided PIDConfig"""
    if feedback_device is not None:
        safe_set_rev_in_thread(pid.setFeedbackDevice, feedback_device)

    safe_set_rev_in_thread(pid.setP, pid_config.p)
    safe_set_rev_in_thread(pid.setI, pid_config.i)
    safe_set_rev_in_thread(pid.setD, pid_config.d)

    if pid_config.wrapping is not None and \
            (pid_config.wrapping.min is not None or pid_config.wrapping.max is not None):
        safe_set_rev_in_thread(pid.setPositionPIDWrappingEnabled, True)
        if pid_config.wrapping.min is not None:
            safe_set_rev_in_thread(pid.setPositionPIDWrappingMinInput, pid_config.wrapping.min)
        if pid_config.wrapping.max is not None:
            safe_set_rev_in_thread(pid.setPositionPIDWrappingMaxInput, pid_config.wrapping.max)

    else:
        safe_set_rev_in_thread(pid.setPositionPIDWrappingEnabled, False)


def read_pid(pid: rev.SparkMaxPIDController) -> PIDConfig:
    """Configures a SparkMax PID controller with the provided PIDConfig"""
    wrapping = pid.getPositionPIDWrappingEnabled()
    if wrapping:
        min_wrap = pid.getPositionPIDWrappingMinInput()
        max_wrap = pid.getPositionPIDWrappingMaxInput()

    return PIDConfig(p=pid.getP(), i=pid.getI(), d=pid.getD(),
                     wrapping=config.OptionalRange(min_wrap, max_wrap) if wrapping else None)


def adjust_pid(pid: rev.SparkMaxPIDController, pid_config: PIDConfig):
    """Configures a SparkMax PID controller with the provided PIDConfig"""
    safe_set_rev_in_thread(pid.setP, pid_config.p)
    safe_set_rev_in_thread(pid.setI, pid_config.i)
    safe_set_rev_in_thread(pid.setD, pid_config.d)


def init_motor(motor: rev.CANSparkMax, config: MotorConfig):
    motor.restoreFactoryDefaults()
    safe_set_rev_in_thread(motor.setIdleMode, rev.CANSparkMax.IdleMode.kCoast)
    safe_set_rev_in_thread(motor.setInverted, config.inverted)
    if config.open_ramp_rate is not None:
        safe_set_rev_in_thread(motor.setOpenLoopRampRate, config.open_ramp_rate)
    if config.closed_ramp_rate is not None:
        safe_set_rev_in_thread(motor.setOpenLoopRampRate, config.closed_ramp_rate)
    if config.current_limit is not None:
        safe_set_rev_in_thread(motor.setOpenLoopRampRate, config.current_limit)


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
        constraints=TrapezoidProfile.Constraints(pid_config.profile.velocity, pid_config.profile.acceleration))

    if pid_config.wrapping is not None:
        pid.enableContinuousInput(minimumInput=pid_config.wrapping.min,
                                  maximumInput=pid_config.wrapping.max)
    if pid_config.tolerance is not None:
        pid.setTolerance(positionTolerance=pid_config.tolerance.position,
                         velocityTolerance=pid_config.tolerance.velocity)
    return pid


def create_pid(pid_config: PIDConfig) -> PIDController:
    pid = PIDController(
        Kp=pid_config.p,
        Ki=pid_config.i,
        Kd=pid_config.d)

    if pid_config.tolerance is not None:
        pid.setTolerance(positionTolerance=pid_config.tolerance.position,
                         velocityTolerance=pid_config.tolerance.velocity)

    return pid
