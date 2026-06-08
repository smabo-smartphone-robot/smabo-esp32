"""Generic servo-joint controller backed by a PCA9685.

All PCA9685-driven joints (head, hands, arm, …) share this one class.
Single-point and multi-point JointTrajectory messages are handled uniformly
via load_trajectory() so that MoveIt2 can drive the arm through smooth,
time-respecting motions.

Trajectory execution model
──────────────────────────
load_trajectory() handles both single-point and multi-point JointTrajectory
messages uniformly.  Each point is stored with an absolute wall-clock
deadline (start_ms + time_from_start).  The run() loop advances through the
queue whenever the current time reaches a point's deadline, then updates
_target for the affected joints.  The per-joint max_speed limiter moves
_current smoothly toward _target each tick.

Random motion uses set_angle_deg() which updates _target directly,
bypassing the trajectory queue entirely.
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


class JointGroup:
    """Drives a named set of servo joints on a single PCA9685."""

    def __init__(self, pca, joints):
        """Snap every joint to its init_angle and prepare the trajectory queue.

        Parameters
        ----------
        pca : pca9685.PCA9685
            The PWM controller the joints are wired to.
        joints : dict
            Mapping of joint name to servo spec dict (see ``config._servo``).
        """
        self.pca = pca
        self.joints = joints        # {name: servo_spec_dict}
        self._target  = {}          # deg — where each joint is heading
        self._current = {}          # deg — last commanded position
        self._traj_queue = []       # [(abs_deadline_ms, {name: deg})]
        self._traj_idx   = 0        # next unprocessed queue entry
        # Transient per-move speed override (deg/s) used by random motion so it
        # can vary speed without touching the configured max_speed. Cleared on
        # the next manual move of that joint.
        self._speed_override = {}
        for name, spec in joints.items():
            init = spec.get("init_angle", 0)
            self._target[name]  = init
            self._current[name] = init
            self._apply(name, init)

    # -- low level ---------------------------------------------------------
    def _apply(self, name, angle_deg):
        """Convert a clamped angle to a pulse width and push it to the PCA9685.

        Updates the cached current position for the joint.

        Parameters
        ----------
        name : str
            Joint name (must exist in ``self.joints``).
        angle_deg : float
            Desired angle in degrees; clamped to the joint's limits.

        Returns
        -------
        None
        """
        spec  = self.joints[name]
        angle = _clamp(angle_deg, spec["min_angle"], spec["max_angle"])
        span  = spec["max_angle"] - spec["min_angle"]
        ratio = (angle - spec["min_angle"]) / span if span else 0.0
        us    = spec["min_us"] + ratio * (spec["max_us"] - spec["min_us"])
        self.pca.set_us(spec["channel"], us)
        self._current[name] = angle

    # -- single-point API (manual control / random motion) ----------------
    def set_angle_deg(self, name, angle, speed=None):
        """Set a joint's target angle in degrees.

        Unknown joint names are ignored. The follower ramps toward the target
        at ``speed`` (deg/s) when given — a transient override used by random
        motion — otherwise at the joint's configured ``max_speed``. A move with
        an effective speed of 0 is applied immediately. Passing ``speed=None``
        also clears any previous override (so manual moves use the config).

        Parameters
        ----------
        name : str
            Joint name.
        angle : float
            Target angle in degrees; clamped to the joint's limits.
        speed : float, optional
            Per-move speed override in deg/s (default None = use config).

        Returns
        -------
        None
        """
        if name not in self.joints:
            return
        self._target[name] = _clamp(
            angle, self.joints[name]["min_angle"], self.joints[name]["max_angle"]
        )
        if speed is None:
            self._speed_override.pop(name, None)
        else:
            self._speed_override[name] = speed
        eff = speed if speed is not None else self.joints[name].get("max_speed", 0)
        if eff <= 0:
            self._apply(name, self._target[name])

    # -- trajectory API (single or multi-point) ----------------------------
    def load_trajectory(self, joint_names, points):
        """Load a JointTrajectory — 1 point or many, handled identically.

        Each point is scheduled at ``ticks_ms() + time_from_start`` and applied
        by :meth:`run` when its deadline passes. A single point with
        ``time_from_start = 0`` is applied on the next tick (≤20 ms), which is
        effectively immediate. Replaces any queued trajectory.

        Parameters
        ----------
        joint_names : list of str
            Joint names corresponding positionally to each point's positions.
        points : list of dict
            JointTrajectory points; each has ``positions`` (radians) and an
            optional ``time_from_start`` with ``sec`` / ``nanosec``.

        Returns
        -------
        None
        """
        if not points:
            return
        start_ms = _ticks_ms()
        queue = []
        for pt in points:
            tfs   = pt.get("time_from_start") or {}
            t_ms  = int(tfs.get("sec", 0) * 1000
                        + tfs.get("nanosec", 0) // 1_000_000)
            targets = {}
            for name, pos in zip(joint_names, pt.get("positions") or []):
                if name in self.joints:
                    targets[name] = pos * 180.0 / math.pi   # rad → deg
                    self._speed_override.pop(name, None)     # use config speed
            if targets:
                queue.append((start_ms + t_ms, targets))
        self._traj_queue = queue
        self._traj_idx   = 0

    # -- state query -------------------------------------------------------
    def get_state(self):
        """Return the current commanded angle of every joint, in radians.

        Returns
        -------
        dict
            Mapping of joint name to current angle in radians.
        """
        return {n: a * math.pi / 180.0 for n, a in self._current.items()}

    # -- async run loop ----------------------------------------------------
    async def run(self, dt=0.02):
        """Run the follower loop: advance the trajectory queue and ramp joints.

        Each tick, points whose deadline has passed update their targets, then
        every joint with a non-zero ``max_speed`` steps toward its target.

        Parameters
        ----------
        dt : float, optional
            Loop period in seconds (default 0.02 = 50 Hz).

        Returns
        -------
        None
            Never returns (cancelled when the servo subsystem restarts).
        """
        while True:
            now = _ticks_ms()

            # Advance through trajectory points whose deadline has passed.
            while self._traj_idx < len(self._traj_queue):
                deadline_ms, targets = self._traj_queue[self._traj_idx]
                if _ticks_diff(now, deadline_ms) >= 0:
                    for name, deg in targets.items():
                        spec = self.joints.get(name)
                        if spec:
                            self._target[name] = _clamp(
                                deg, spec["min_angle"], spec["max_angle"]
                            )
                    self._traj_idx += 1
                else:
                    break   # remaining points are still in the future

            # Speed-limited follower: move _current toward _target. A transient
            # override (set by random motion) takes precedence over max_speed.
            for name, spec in self.joints.items():
                speed = self._speed_override.get(name, spec.get("max_speed", 0))
                if speed <= 0:
                    continue
                cur  = self._current[name]
                tgt  = self._target[name]
                step = speed * dt
                if abs(tgt - cur) <= step:
                    if cur != tgt:
                        self._apply(name, tgt)
                else:
                    self._apply(name, cur + step if tgt > cur else cur - step)

            await asyncio.sleep(dt)
