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

    def drive(self, value):
        """Drive the motor at a signed normalised speed.

        Parameters
        ----------
        value : float
            Signed speed in ``[-1.0, 1.0]``; sign selects direction,
            magnitude selects duty. Near-zero coasts the motor.

        Returns
        -------
        None
        """
        if self.invert:
            value = -value
        duty = int(abs(value) * _DUTY_MAX)
        duty = _clamp(duty, 0, _DUTY_MAX)
        if value > 0.001:
            self.in1.value(1)
            self.in2.value(0)
        elif value < -0.001:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(0)
            self.in2.value(0)
        self.pwm.duty(duty)

    def stop(self):
        """Coast the motor: both direction pins low and 0 % duty.

        Returns
        -------
        None
        """
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
        freq = cfg.get("dc.pwm_freq", 1000)
        self.stby = Pin(p["stby"], Pin.OUT)
        self.stby.value(1)
        self.left = _Motor(p["ain1"], p["ain2"], p["pwma"], freq,
                           cfg.get("dc.invert_left", False))
        self.right = _Motor(p["bin1"], p["bin2"], p["pwmb"], freq,
                            cfg.get("dc.invert_right", False))
        self._last_cmd_ms = _ticks_ms()

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
        max_lin = self.cfg.get("dc.max_linear", 0.3)
        max_ang = self.cfg.get("dc.max_angular", 1.5)
        sep = self.cfg.get("dc.wheel_separation", 0.15)

        # differential kinematics, normalised to drive units
        v_l = linear_x - angular_z * sep / 2.0
        v_r = linear_x + angular_z * sep / 2.0
        # normalise by the worst-case full-speed magnitude
        norm = max_lin + max_ang * sep / 2.0
        if norm <= 0:
            norm = 1.0
        self._last_cmd_ms = _ticks_ms()
        self.left.drive(_clamp(v_l / norm, -1.0, 1.0))
        self.right.drive(_clamp(v_r / norm, -1.0, 1.0))

    def stop(self):
        """Coast both wheels.

        Returns
        -------
        None
        """
        self.left.stop()
        self.right.stop()

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
            timeout_ms = int(self.cfg.get("dc.cmd_timeout", 0.5) * 1000)
            if _ticks_diff(_ticks_ms(), self._last_cmd_ms) > timeout_ms:
                self.stop()
            await asyncio.sleep_ms(100)
