"""Wheel velocity publisher.

Reads encoder deltas at the configured rate and publishes /wheel_vel
(left and right wheel speeds in m/s + the actual integration interval dt).
Pose integration is done by smabo-brain so it can be fused with IMU/GPS.
"""

import math

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

try:
    import time
    _ticks_ms   = time.ticks_ms
    _ticks_diff = time.ticks_diff
except AttributeError:
    import time
    _ticks_ms   = lambda: int(time.time() * 1000)
    _ticks_diff = lambda a, b: a - b


class WheelPublisher:
    """Reads encoders and publishes /wheel_vel for brain-side integration."""

    def __init__(self, cfg, enc_left, enc_right, on_measure=None):
        """``on_measure(left, right, dt)`` gets every sample (wheel speed control)."""
        self.cfg       = cfg
        self.on_measure = on_measure
        self.enc_left  = enc_left
        self.enc_right = enc_right
        self._last_ms  = _ticks_ms()

    def _meters_per_count(self):
        r   = self.cfg.get("dc.wheel_radius")
        cpr = self.cfg.get("encoder.cpr")
        return (2.0 * math.pi * r) / cpr

    async def run(self, publish):
        """Loop forever, publishing ``/wheel_vel`` at the configured rate."""
        while True:
            rate = self.cfg.get("encoder.publish_rate")
            await asyncio.sleep(1.0 / rate if rate > 0 else 0.05)

            now       = _ticks_ms()
            actual_dt = _ticks_diff(now, self._last_ms) / 1000.0
            self._last_ms = now
            if actual_dt <= 0:
                continue

            mpc = self._meters_per_count()
            d_l = self.enc_left.read_and_reset()  * mpc
            d_r = self.enc_right.read_and_reset() * mpc

            v_l, v_r = d_l / actual_dt, d_r / actual_dt
            if self.on_measure is not None:
                self.on_measure(v_l, v_r, actual_dt)
            publish("/wheel_vel", {
                "left":  v_l,
                "right": v_r,
                "dt":    actual_dt,
            })
