"""Differential-drive DC motor control via a TB6612FNG (or compatible).

Uses the ESP32 native LEDC PWM (machine.PWM) at ~1 kHz, kept separate from
the PCA9685 servo PWM on purpose.  Consumes geometry_msgs/Twist (cmd_vel)
and applies a dead-man timeout so the robot stops if commands stop arriving.
"""

from machine import Pin, PWM

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

try:
    import time
    _ticks_ms = time.ticks_ms
    _ticks_diff = time.ticks_diff
except AttributeError:
    import time
    _ticks_ms = lambda: int(time.time() * 1000)
    _ticks_diff = lambda a, b: a - b

_DUTY_MAX = 1023  # 10-bit duty on ESP32 LEDC
_COAST = 0.001    # |duty| at or below this coasts the motor (see _Motor.drive)
_HOLD_DUTY = 0.01  # smallest speed-control output: keeps the direction (no brake, no re-kick)
_STOP_SPEED = 1e-4  # m/s; a wheel speed target below this means "stop"


def _clamp(v, lo, hi):
    """Clamp a value to an inclusive range.

    Parameters
    ----------
    v : float
        Value to clamp.
    lo : float
        Lower bound.
    hi : float
        Upper bound.

    Returns
    -------
    float
        ``v`` constrained to ``[lo, hi]``.
    """
    return lo if v < lo else hi if v > hi else v


def _with_min_duty(duty, min_duty):
    """Raise a non-zero duty to at least ``min_duty`` (static-friction floor).

    Low duties (e.g. slow in-place rotation) may not overcome static friction.
    ``dc.min_duty`` lifts any non-zero command to that floor while keeping its
    sign; a duty the motor treats as zero (|duty| <= 0.001, see ``_Motor.drive``)
    stays zero so the robot still stops. 0 disables the floor.

    Parameters
    ----------
    duty : float
        Normalised duty in [-1, 1].
    min_duty : float
        Floor in [0, 1].

    Returns
    -------
    float
        The duty with the floor applied.
    """
    if abs(duty) <= _COAST or min_duty <= 0.0 or abs(duty) >= min_duty:
        return duty
    return min_duty if duty > 0 else -min_duty


class _Motor:
    """One H-bridge channel: two direction pins plus a PWM speed pin."""

    def __init__(self, in1, in2, pwm_pin, freq, invert=False):
        """Configure the direction pins and start the PWM at 0 % duty.

        Parameters
        ----------
        in1 : int
            GPIO number of the first H-bridge direction pin.
        in2 : int
            GPIO number of the second H-bridge direction pin.
        pwm_pin : int
            GPIO number driving the PWM speed input.
        freq : int
            PWM frequency in Hz.
        invert : bool, optional
            If True, flip the sign of every drive command (default False).
        """
        self.in1 = Pin(in1, Pin.OUT)
        self.in2 = Pin(in2, Pin.OUT)
        self.pwm = PWM(Pin(pwm_pin), freq=freq)
        self.pwm.duty(0)
        self.invert = invert
        self._dir = 0                    # sign of the last drive, 0 = stopped
        self._kick_from = 0

    def drive(self, value, start_duty=0.0, kick_ms=0, brake=False):
        """Drive the motor at a signed normalised speed.

        Parameters
        ----------
        value : float
            Signed speed in ``[-1.0, 1.0]``; sign selects direction,
            magnitude selects duty. Near-zero stops the motor.
        start_duty : float, optional
            Kick-start: for ``kick_ms`` after the motor starts from rest or
            reverses, the duty is at least this (breaks static friction, which
            needs far more duty than keeping a wheel turning). 0 = off.
        kick_ms : int, optional
            Length of the kick-start.
        brake : bool, optional
            Near-zero short-brakes the motor instead of letting it coast.

        Returns
        -------
        None
        """
        if self.invert:
            value = -value
        if -_COAST <= value <= _COAST:
            self.stop(brake)
            return
        sign = 1 if value > 0 else -1
        now = _ticks_ms()
        if sign != self._dir:            # from rest, or reversing
            self._dir = sign
            self._kick_from = now
        mag = abs(value)
        if start_duty > mag and _ticks_diff(now, self._kick_from) < kick_ms:
            mag = start_duty
        duty = _clamp(int(mag * _DUTY_MAX), 0, _DUTY_MAX)
        if sign > 0:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)
        self.pwm.duty(duty)

    def stop(self, brake=False):
        """Stop the motor: short-brake (both inputs high), or coast (both low).

        Returns
        -------
        None
        """
        self._dir = 0
        if brake:
            # TB6612FNG: IN1 = IN2 = H shorts the motor whatever PWM is
            self.in1.value(1)
            self.in2.value(1)
            self.pwm.duty(_DUTY_MAX)
        else:
            self.in1.value(0)
            self.in2.value(0)
            self.pwm.duty(0)


class DiffDrive:
    """Two-wheel differential drive controlled by geometry_msgs/Twist."""

    def __init__(self, cfg):
        """Build both motors from the ``dc`` config section and enable STBY.

        Parameters
        ----------
        cfg : config.Config
            The shared configuration instance; the ``dc`` section supplies
            pins, PWM frequency and inversion flags.
        """
        self.cfg = cfg
        p = cfg.get("dc.pins")
        freq = cfg.get("dc.pwm_freq")
        self.stby = Pin(p["stby"], Pin.OUT)
        self.stby.value(1)
        self.left = _Motor(p["ain1"], p["ain2"], p["pwma"], freq,
                           cfg.get("dc.invert_left"))
        self.right = _Motor(p["bin1"], p["bin2"], p["pwmb"], freq,
                            cfg.get("dc.invert_right"))
        self._last_cmd_ms = _ticks_ms()
        # Wheel speed control (encoder_drive): target wheel speeds in m/s and
        # the PI integrators, in duty units. See update_speed().
        self.closed_loop = False
        self._target = (0.0, 0.0)
        self._integ = [0.0, 0.0]
        # |integrator| each wheel needed when it last ran: friction learned on
        # the robot, used to start the next run near the right duty
        self._learned = [0.0, 0.0]

    def set_closed_loop(self, on):
        """Switch between open-loop duty mixing and encoder speed control.

        Parameters
        ----------
        on : bool
            True when wheel speeds are measured (encoder_drive) and fed to
            ``update_speed``; also requires ``dc.speed_control`` (default True).

        Returns
        -------
        None
        """
        self.closed_loop = bool(on)
        self._target = (0.0, 0.0)
        self._integ = [0.0, 0.0]

    def _speed_control(self):
        """True if wheel speeds are being regulated from encoder feedback."""
        return self.closed_loop and self.cfg.get("dc.speed_control")

    def _full_duty_speed(self):
        """Wheel speed (m/s) mapped to duty 1.0: ``max_linear + max_angular * sep / 2``."""
        norm = (self.cfg.get("dc.max_linear")
                + self.cfg.get("dc.max_angular") * self.cfg.get("dc.wheel_separation") / 2.0)
        return norm if norm > 0 else 1.0

    def _feedforward(self):
        """Duty per m/s of wheel speed: the open-loop mapping."""
        return 1.0 / self._full_duty_speed()

    def set_cmd_vel(self, linear_x, angular_z):
        """Map a Twist to normalised left/right wheel commands and apply them.

        Also refreshes the dead-man timer so the watchdog keeps the motors
        enabled.

        Parameters
        ----------
        linear_x : float
            Forward velocity command in m/s.
        angular_z : float
            Yaw rate command in rad/s (positive = counter-clockwise).

        Returns
        -------
        None
        """
        sep = self.cfg.get("dc.wheel_separation")
        min_duty = self.cfg.get("dc.min_duty")

        self._last_cmd_ms = _ticks_ms()

        # Differential kinematics for every command, pure spin included, so the
        # robot turns at the commanded rate (Nav2 relies on that). The former
        # pure-spin special case drove |w|/max_angular as duty, which turned
        # ~4x faster than commanded and made Nav2 overshoot and oscillate.
        v_l = linear_x - angular_z * sep / 2.0
        v_r = linear_x + angular_z * sep / 2.0
        if self._speed_control():
            self._set_target(v_l, v_r)
            return
        norm = self._full_duty_speed()
        start = self.cfg.get("dc.start_duty")
        kick = int(self.cfg.get("dc.kick_ms"))
        brake = self.cfg.get("dc.brake")
        self.left.drive(_with_min_duty(_clamp(v_l / norm, -1.0, 1.0), min_duty), start, kick, brake)
        self.right.drive(_with_min_duty(_clamp(v_r / norm, -1.0, 1.0), min_duty), start, kick, brake)

    def _set_target(self, v_l, v_r):
        """Set the wheel speed targets (m/s); start or stop the wheels at once.

        A wheel that is stopped, or asked to reverse, starts from the
        feed-forward duty right away instead of waiting for the next encoder
        sample; a zero target brakes it.
        """
        old = self._target
        self._target = (v_l, v_r)
        ff = self._feedforward()
        for i, (motor, v) in enumerate(((self.left, v_l), (self.right, v_r))):
            if abs(v) < _STOP_SPEED:
                self._remember(i)
                motor.stop(self.cfg.get("dc.brake"))
            elif old[i] * v <= 0:            # from rest, or reversing
                self._remember(i)
                self._integ[i] = self._learned[i] if v > 0 else -self._learned[i]
                self._drive(motor, v * ff + self._integ[i])

    def _remember(self, i):
        """Keep the friction the integrator found on the run that just ended."""
        if self._integ[i] != 0.0:
            self._learned[i] = abs(self._integ[i])
        self._integ[i] = 0.0

    def _drive(self, motor, duty):
        """Drive one motor under speed control: no min_duty floor, no kick-start
        (a kick overshoots; the learned integrator starts the wheel instead)."""
        motor.drive(_clamp(duty, -1.0, 1.0), 0.0, 0, self.cfg.get("dc.brake"))

    def update_speed(self, meas_l, meas_r, dt):
        """PI step: correct each wheel's duty from its measured speed.

        duty = ff * target + kp * error + I,   I += ki * error * dt

        ``ff`` is the open-loop mapping (``dc.max_linear``/``max_angular``), so
        the integrator only has to make up for friction and battery voltage. It
        is clamped to ``dc.speed_i_max`` so a dead encoder cannot run a wheel
        at full duty. Called by the wheel publisher after every encoder sample.

        Parameters
        ----------
        meas_l, meas_r : float
            Measured wheel speeds in m/s (+ = forward).
        dt : float
            Seconds since the previous sample.

        Returns
        -------
        None
        """
        if not self._speed_control() or dt <= 0:
            return
        kp = self.cfg.get("dc.speed_kp")
        ki = self.cfg.get("dc.speed_ki")
        i_max = self.cfg.get("dc.speed_i_max")
        ff = self._feedforward()
        for i, (motor, v, m) in enumerate(((self.left, self._target[0], meas_l),
                                           (self.right, self._target[1], meas_r))):
            if abs(v) < _STOP_SPEED:
                continue                      # braked in _set_target / stop
            err = v - m
            # Anti-windup: never let the integrator cancel more than the
            # feed-forward (it would then have to unwind before the wheel moves).
            lo, hi = (-v * ff, i_max) if v > 0 else (-i_max, -v * ff)
            self._integ[i] = _clamp(self._integ[i] + ki * err * dt, lo, hi)
            duty = v * ff + kp * err + self._integ[i]
            # Never drive against the target, and never let the output reach
            # zero: _Motor.drive would brake and then kick-start again next
            # sample, a stop/kick cycle that runs the wheel in bursts.
            sign = 1.0 if v > 0 else -1.0
            if duty * sign < _HOLD_DUTY:
                duty = _HOLD_DUTY * sign
            self._drive(motor, duty)

    def stop(self):
        """Stop both wheels: short-brake by default (``dc.brake``), else coast.

        Braking stops a turning wheel in a fraction of the time coasting takes.

        Returns
        -------
        None
        """
        brake = self.cfg.get("dc.brake")
        self._target = (0.0, 0.0)
        self._remember(0)
        self._remember(1)
        self.left.stop(brake)
        self.right.stop(brake)

    async def watchdog_task(self):
        """Stop the motors whenever no cmd_vel has arrived within the timeout.

        Runs forever, polling every 100 ms; the timeout is read live from
        ``dc.cmd_timeout``.

        Returns
        -------
        None
            Never returns (cancelled by the orchestrator).
        """
        while True:
            timeout_ms = int(self.cfg.get("dc.cmd_timeout") * 1000)
            if _ticks_diff(_ticks_ms(), self._last_cmd_ms) > timeout_ms:
                self.stop()
            await asyncio.sleep_ms(100)
