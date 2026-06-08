"""Differential-drive odometry from wheel encoders.

Integrates wheel displacement into an (x, y, theta) pose and produces a
nav_msgs/Odometry-shaped dict ready to be published over the rosbridge
protocol.

Covariance matrices are built from five diagonal variances stored in
cfg.encoder.covariance so Nav2 can weight odometry correctly.
Non-planar DoF (z, roll, pitch) get a large fixed variance (1e6) to signal
"not measured by this sensor".
"""

import math

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

_BIG = 1e6  # variance for unmeasured DoF (z, roll, pitch)


def _pose_cov(cfg):
    """Build the 6x6 pose covariance (row-major) from configured variances.

    Unmeasured DoF (z, roll, pitch) are filled with a large variance.

    Parameters
    ----------
    cfg : config.Config
        Shared config; reads ``encoder.covariance`` diagonal variances.

    Returns
    -------
    list of float
        A flat 36-element row-major covariance matrix.
    """
    c = cfg.get("encoder.covariance") or {}
    xx = c.get("pose_xx", 0.001)
    yy = c.get("pose_yy", 0.001)
    aa = c.get("pose_aa", 0.001)
    cov = [0.0] * 36
    cov[0]  = xx    # x
    cov[7]  = yy    # y
    cov[14] = _BIG  # z  — unmeasured
    cov[21] = _BIG  # roll  — unmeasured
    cov[28] = _BIG  # pitch — unmeasured
    cov[35] = aa    # yaw
    return cov


def _twist_cov(cfg):
    """Build the 6x6 twist covariance (row-major) from configured variances.

    Unmeasured DoF (vy, vz, wx, wy) are filled with a large variance.

    Parameters
    ----------
    cfg : config.Config
        Shared config; reads ``encoder.covariance`` diagonal variances.

    Returns
    -------
    list of float
        A flat 36-element row-major covariance matrix.
    """
    c = cfg.get("encoder.covariance") or {}
    vv = c.get("twist_vv", 0.001)
    ww = c.get("twist_ww", 0.001)
    cov = [0.0] * 36
    cov[0]  = vv    # vx
    cov[7]  = _BIG  # vy  — unmeasured
    cov[14] = _BIG  # vz  — unmeasured
    cov[21] = _BIG  # wx  — unmeasured
    cov[28] = _BIG  # wy  — unmeasured
    cov[35] = ww    # wz
    return cov


class Odometry:
    """Integrates wheel encoder deltas into a planar pose and Odometry message."""

    def __init__(self, cfg, enc_left, enc_right):
        """Initialise pose/velocity state to zero.

        Parameters
        ----------
        cfg : config.Config
            The shared configuration instance.
        enc_left : encoder.QuadEncoder
            The left wheel encoder.
        enc_right : encoder.QuadEncoder
            The right wheel encoder.
        """
        self.cfg = cfg
        self.enc_left = enc_left
        self.enc_right = enc_right
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.wz = 0.0
        self._last_ms = _ticks_ms()

    def _meters_per_count(self):
        """Compute the wheel travel per encoder count.

        Returns
        -------
        float
            Linear distance in metres advanced per single encoder count.
        """
        r = self.cfg.get("dc.wheel_radius", 0.03)
        cpr = self.cfg.get("encoder.cpr", 1440)
        return (2.0 * math.pi * r) / cpr

    def update(self):
        """Read both encoders and integrate one step of pose and velocity.

        No-op if the elapsed time since the last call is non-positive.

        Returns
        -------
        None
        """
        now = _ticks_ms()
        dt = _ticks_diff(now, self._last_ms) / 1000.0
        self._last_ms = now
        if dt <= 0:
            return

        mpc = self._meters_per_count()
        d_l = self.enc_left.read_and_reset() * mpc
        d_r = self.enc_right.read_and_reset() * mpc
        sep = self.cfg.get("dc.wheel_separation", 0.15)

        d_center = (d_l + d_r) / 2.0
        d_theta = (d_r - d_l) / sep

        self.x += d_center * math.cos(self.theta + d_theta / 2.0)
        self.y += d_center * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.vx = d_center / dt
        self.wz = d_theta / dt

    def odom_msg(self):
        """Build a nav_msgs/Odometry message from the current state.

        Returns
        -------
        dict
            An Odometry-shaped dict with header, pose (+covariance) and twist
            (+covariance), ready to publish over the rosbridge protocol.
        """
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        now_ms = _ticks_ms()
        stamp = {"sec": now_ms // 1000, "nanosec": (now_ms % 1000) * 1_000_000}
        return {
            "header": {
                "stamp": stamp,
                "frame_id": self.cfg.get("encoder.odom_frame", "odom"),
            },
            "child_frame_id": self.cfg.get("encoder.base_frame", "base_link"),
            "pose": {
                "pose": {
                    "position":    {"x": self.x, "y": self.y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": qz, "w": qw},
                },
                "covariance": _pose_cov(self.cfg),
            },
            "twist": {
                "twist": {
                    "linear":  {"x": self.vx, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": self.wz},
                },
                "covariance": _twist_cov(self.cfg),
            },
        }

    async def run(self, publish):
        """Loop forever, integrating and publishing ``/odom`` at the set rate.

        Parameters
        ----------
        publish : callable
            ``publish(topic, msg)`` used to emit each Odometry message.

        Returns
        -------
        None
            Never returns (cancelled when the drive subsystem restarts).
        """
        topic = "/odom"
        while True:
            rate = self.cfg.get("encoder.publish_rate", 20.0)
            self.update()
            try:
                publish(topic, self.odom_msg())
            except Exception as e:
                print("odom publish failed:", e)
            await asyncio.sleep(1.0 / rate if rate > 0 else 0.05)
