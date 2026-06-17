"""Persistent configuration store.

Settings are kept in RAM and mirrored to ``config.json`` in flash.
Writes are debounced so that frequently-changing values do not wear out
the flash.  Anything reachable over WebSocket should go through here so
that it survives a reboot.
"""

import json

try:
    import uasyncio as asyncio
except ImportError:  # CPython fallback for off-target testing
    import asyncio

try:
    import time
    _ticks_ms = time.ticks_ms
    _ticks_diff = time.ticks_diff
except AttributeError:  # CPython
    import time
    _ticks_ms = lambda: int(time.time() * 1000)
    _ticks_diff = lambda a, b: a - b

_PATH = "config.json"
_DEBOUNCE_MS = 2000  # delay before a changed config is flushed to flash

# ---------------------------------------------------------------------------
# Default configuration.  Every tunable lives here; the JSON file only stores
# the *differences* applied at runtime (a deep-merge happens on load).
# ---------------------------------------------------------------------------


def _servo(channel, lo, hi, init=0, max_speed=120):
    """Build a servo spec dict with the standard 500-2500 µs pulse range.

    Parameters
    ----------
    channel : int
        PCA9685 output channel (0-15).
    lo : float
        Minimum angle in degrees.
    hi : float
        Maximum angle in degrees.
    init : float, optional
        Angle commanded at boot (default 0).
    max_speed : float, optional
        Angular speed limit in deg/s; 0 means jump instantly (default 120).

    Returns
    -------
    dict
        A servo spec with keys ``channel``, ``min_angle``, ``max_angle``,
        ``min_us``, ``max_us``, ``init_angle`` and ``max_speed``.
    """
    return {
        "channel": channel,
        "min_angle": lo,
        "max_angle": hi,
        "min_us": 500,
        "max_us": 2500,
        "init_angle": init,
        "max_speed": max_speed,  # deg/s, 0 = jump instantly
    }


DEFAULTS = {
    "wifi": {
        "ssid": "your-ssid",
        "password": "your-password",
        "hostname": "esp32-robot",
    },
    # smabo-brain relay server to connect to
    "brain": {
        "host": "192.168.1.100",
        "port": 9090,
    },
    "i2c": {"sda": 21, "scl": 22, "freq": 400000},  # classic ESP32 default
    "pca9685": {"address": 0x40, "freq": 50},

    # Which subsystems are active. Multiple may run at once, EXCEPT
    # dc_drive and encoder_drive are mutually exclusive (enforced at runtime).
    "modes": {
        "servos": True,
        "dc_drive": False,
        "encoder_drive": False,
    },

    # -----------------------------------------------------------------------
    # Servo subsystem: ALL PCA9685-driven joints (head, hands, arm, extras).
    #
    # random_groups defines which joints fire together.  Joints in the same
    # group move at the same instant (each to its own random angle); groups
    # run on independent timers.  A joint not listed in any group is only
    # reachable via manual /servo/command messages.
    # -----------------------------------------------------------------------
    "servos": {
        "behavior": "manual",   # "manual" | "random"
        "joints": {
            # ch 0-1: hand (2 servos)
            "left_hand":   _servo(0,   0, 90, 0,   0),
            "right_hand":  _servo(1,   0, 90, 0,   0),
            # ch 2: neck pan only (tilt removed)
            "head_pan":    _servo(2, -90, 90, 0, 120),
            # ch 3-6: arm joints — add/remove to match the number of axes on your robot
            "arm_joint_1": _servo(3, -90, 90, 0,  90),
            "arm_joint_2": _servo(4, -90, 90, 0,  90),
            "arm_joint_3": _servo(5, -90, 90, 0,  90),
            "arm_joint_4": _servo(6, -90, 90, 0,  90),
        },
        "random_groups": [
            # joints listed together fire simultaneously (angles are random per joint)
            {"name": "hands", "joints": ["left_hand", "right_hand"], "interval": [2.0, 5.0]},
            {"name": "neck",  "joints": ["head_pan"], "interval": [1.0, 3.0]},
        ],
        # Rate at which /joint_states is published (required by MoveIt2).
        # Set to 0 to disable.
        "joint_states_rate": 20.0,
    },

    "dc": {
        # Default targets the classic (non-S3) ESP32 DevKit. These 7 pads sit
        # in one contiguous run on the left header, so a ribbon of still-joined
        # jumper wires plugs straight across the TB6612FNG control header in the
        # breakout's physical order:
        #   GPIO 32=PWMA, 33=AIN2, 25=AIN1, 26=STBY, 27=BIN1, 14=BIN2, 12=PWMB
        # Input-only pins (34/35/36/39) are avoided since motor outputs need
        # drive capability. For ESP32-S3 / XIAO boards apply a config.json from
        # configs/ instead.
        "pins": {
            "pwma": 32, "ain2": 33, "ain1": 25,
            "stby": 26,
            "bin1": 27, "bin2": 14, "pwmb": 12,
        },
        "pwm_freq": 1000,
        "max_linear": 0.30,        # m/s mapped to full duty
        "max_angular": 1.50,       # rad/s mapped to full duty
        "wheel_radius": 0.030,     # m
        "wheel_separation": 0.150,  # m (track width)
        "invert_left": False,
        "invert_right": False,
        "cmd_timeout": 0.5,        # s; stop motors if no cmd_vel within this
    },

    "encoder": {
        "left": {"a": 34, "b": 35},
        "right": {"a": 36, "b": 39},
        "cpr": 1440,               # encoder counts per *wheel* revolution
        "publish_rate": 20.0,      # Hz for /odom
        "odom_frame": "odom",
        "base_frame": "base_link",
        # Diagonal variances for nav_msgs/Odometry covariance matrices.
        # Non-planar DoF (z, roll, pitch) are fixed at 1e6 (unmeasured).
        "covariance": {
            "pose_xx":   0.001,   # x position variance (m^2)
            "pose_yy":   0.001,   # y position variance (m^2)
            "pose_aa":   0.001,   # yaw variance (rad^2)
            "twist_vv":  0.001,   # linear velocity variance ((m/s)^2)
            "twist_ww":  0.001,   # angular velocity variance ((rad/s)^2)
        },
    },
}


def _deep_merge(base, override):
    """Recursively merge ``override`` onto ``base`` without mutating either.

    Parameters
    ----------
    base : dict
        The base mapping whose values are used where not overridden.
    override : dict
        The mapping whose values take precedence; nested dicts are merged
        recursively, and keys present only here are added.

    Returns
    -------
    dict
        A new merged dictionary.
    """
    out = {}
    for k, v in base.items():
        if k in override and isinstance(v, dict) and isinstance(override[k], dict):
            out[k] = _deep_merge(v, override[k])
        elif k in override:
            out[k] = override[k]
        else:
            out[k] = v
    # keys present only in override (e.g. extra arm joints)
    for k, v in override.items():
        if k not in out:
            out[k] = v
    return out


class Config:
    """In-RAM config mirrored to flash, with deep-merge and debounced saves."""

    def __init__(self, path=_PATH):
        """Load ``path`` (if present) merged over the built-in DEFAULTS.

        Parameters
        ----------
        path : str, optional
            Path to the JSON config file in flash (default ``config.json``).
        """
        self._path = path
        self._dirty = False
        self._save_at = None
        self.data = DEFAULTS
        self._load()

    # -- persistence -------------------------------------------------------
    def _load(self):
        """Read the JSON file and deep-merge it over DEFAULTS.

        Falls back to pure DEFAULTS if the file is missing or invalid.

        Returns
        -------
        None
        """
        try:
            with open(self._path) as f:
                stored = json.load(f)
            self.data = _deep_merge(DEFAULTS, stored)
        except (OSError, ValueError):
            self.data = _deep_merge(DEFAULTS, {})

    def _write_now(self):
        """Serialise the live config to flash and clear the dirty flag.

        Returns
        -------
        None
        """
        try:
            with open(self._path, "w") as f:
                json.dump(self.data, f)
            self._dirty = False
        except OSError as e:
            print("config: write failed", e)

    def save_now(self):
        """Flush pending changes to flash immediately (use before reset).

        Returns
        -------
        None
        """
        self._write_now()

    def mark_dirty(self):
        """Flag pending changes and (re)arm the debounce timer for autosave.

        Returns
        -------
        None
        """
        self._dirty = True
        self._save_at = _ticks_ms() + _DEBOUNCE_MS

    async def autosave_task(self):
        """Background loop that flushes pending changes after the debounce.

        Returns
        -------
        None
            Never returns (cancelled or runs for the program lifetime).
        """
        while True:
            await asyncio.sleep_ms(500)
            if self._dirty and self._save_at is not None:
                if _ticks_diff(_ticks_ms(), self._save_at) >= 0:
                    self._write_now()

    # -- dotted-path access ------------------------------------------------
    def get(self, path, default=None):
        """Return the value at a dotted path, or a default if absent.

        Parameters
        ----------
        path : str
            Dotted key path, e.g. ``"dc.pins"``.
        default : object, optional
            Value returned if any segment is missing (default ``None``).

        Returns
        -------
        object
            The value at ``path``, or ``default``.
        """
        node = self.data
        for key in path.split("."):
            if isinstance(node, dict) and key in node:
                node = node[key]
            else:
                return default
        return node

    def update(self, partial):
        """Deep-merge overrides into the live config and schedule a save.

        Parameters
        ----------
        partial : dict
            Nested overrides to merge into the current config.

        Returns
        -------
        None
        """
        self.data = _deep_merge(self.data, partial)
        self.mark_dirty()
