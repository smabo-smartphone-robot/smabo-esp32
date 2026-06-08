"""Group-based, lifelike random motion for servo joints.

Joints are arranged into named timing groups (cfg.servos.random_groups).
Within one group all joints fire at the same instant, each to an independent
target.  Groups run on completely independent timers so they drift apart
naturally.

Rather than jumping to a uniform-random absolute angle, each fire is usually a
subtle, slow drift (small gaussian step pulled back toward the joint's rest
angle) with an occasional quicker, larger "saccade".  Per-move speed varies
(slow drift vs fast glance) and hold times are mostly short with the odd long
settle, so the result reads as organic rather than mechanical.

Config is re-read on every iteration, so these values are hot-reloadable
without restarting the task:
  - behavior (manual ↔ random switch takes effect on the next tick)
  - interval  (new range applies after the current sleep expires)
  - joint membership changes (takes effect on the next fire)

Adding or removing *groups* requires restarting the servos subsystem because
each group maps to its own asyncio task.
"""

import random

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio


class RandomMotion:
    """Manages per-group random motion tasks on top of a JointGroup."""

    def __init__(self, joint_group, cfg):
        """Bind to the servo :class:`JointGroup` and shared config.

        Parameters
        ----------
        joint_group : servo_controller.JointGroup
            The JointGroup whose joints will be moved.
        cfg : config.Config
            The shared config instance (re-read live every iteration).
        """
        self.group = joint_group   # JointGroup instance
        self.cfg = cfg
        self._pos = {}             # joint -> last target angle (random walk)

    # -- config helpers (always fresh) ------------------------------------
    def _behavior(self):
        """Return the current servo behaviour.

        Returns
        -------
        str
            Either ``"manual"`` or ``"random"``.
        """
        return self.cfg.get("servos.behavior", "manual")

    def _group_cfg(self, name):
        """Look up the live config for a named random group.

        Parameters
        ----------
        name : str
            The group's ``name`` field.

        Returns
        -------
        dict or None
            The matching group config, or ``None`` if no such group exists.
        """
        for g in (self.cfg.get("servos.random_groups") or []):
            if g.get("name") == name:
                return g
        return None

    # -- helpers ----------------------------------------------------------
    def _gauss(self, sigma):
        """Approximate a normal sample ~N(0, sigma).

        MicroPython's ``random`` has no ``gauss``; the sum of 12 uniforms minus
        6 is ~N(0, 1) (central-limit), scaled by ``sigma``.

        Parameters
        ----------
        sigma : float
            Standard deviation.

        Returns
        -------
        float
            A pseudo-normal sample.
        """
        s = 0.0
        for _ in range(12):
            s += random.random()
        return (s - 6.0) * sigma

    def _gp(self, g, key, default):
        """Read a numeric per-group tuning parameter, or a default.

        Parameters
        ----------
        g : dict
            The random-group config.
        key : str
            Parameter name.
        default : float
            Value used when missing or non-numeric.

        Returns
        -------
        float
            The parameter value.
        """
        v = g.get(key, default)
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def _center(self, spec):
        """The joint's rest angle: ``init_angle`` if in range, else the midpoint.

        Parameters
        ----------
        spec : dict
            Servo spec (``min_angle``/``max_angle``/``init_angle``).

        Returns
        -------
        float
            The rest/centre angle in degrees.
        """
        lo, hi = spec["min_angle"], spec["max_angle"]
        c = spec.get("init_angle", 0)
        if c < lo or c > hi:
            c = (lo + hi) / 2.0
        return c

    # -- per-group coroutine ----------------------------------------------
    async def run_group(self, group_name):
        """Run a lifelike random-motion loop for one group.

        Most ticks are subtle, mean-reverting drifts (slow, near the rest
        angle); occasionally a quicker, larger "saccade" glances elsewhere.
        Hold times are usually short but sometimes long (settling), so the
        motion reads as organic rather than mechanical. Joints in a group still
        fire on the same tick, each to its own independent angle.

        Parameters
        ----------
        group_name : str
            The name of the random group this task drives.

        Returns
        -------
        None
            Never returns (cancelled when the servo subsystem restarts).
        """
        while True:
            g = self._group_cfg(group_name)

            if g is None or self._behavior() != "random":
                await asyncio.sleep_ms(200)
                continue

            # Per-group tuning (all optional, hot-reloadable, with defaults).
            saccade_prob = self._gp(g, "saccade_prob", 0.18)
            drift_frac   = self._gp(g, "drift", 0.07)
            center_pull  = self._gp(g, "center_pull", 0.12)
            drift_speed  = self._gp(g, "drift_speed", 0.4)
            long_prob    = self._gp(g, "long_pause_prob", 0.22)

            saccade = random.random() < saccade_prob   # quick, larger glance

            # Fire all joints in the group at the same moment.
            for jname in (g.get("joints") or []):
                spec = self.group.joints.get(jname)
                if spec is None:
                    continue
                lo, hi = spec["min_angle"], spec["max_angle"]
                span = hi - lo
                if span <= 0:
                    continue
                center = self._center(spec)
                cur = self._pos.get(jname, center)

                base = spec.get("max_speed", 0)
                if base <= 0:
                    base = 120.0   # smooth default when configured "instant"

                if saccade:
                    # Larger move within a comfortable sub-range, at full speed.
                    margin = span * 0.12
                    target = random.uniform(lo + margin, hi - margin)
                    speed = base
                else:
                    # Subtle drift: small gaussian step pulled toward centre.
                    target = (cur
                              + self._gauss(span * drift_frac)
                              + center_pull * (center - cur)
                              + random.uniform(-1.0, 1.0) * span * 0.01)
                    speed = base * drift_speed * random.uniform(0.7, 1.0)

                if target < lo:
                    target = lo
                elif target > hi:
                    target = hi

                self._pos[jname] = target
                self.group.set_angle_deg(jname, target, speed=speed)

            # Hold: usually short; glance again sooner after a saccade, and now
            # and then hold for a longer, natural settle.
            lo_t, hi_t = g.get("interval") or [1.0, 3.0]
            t = random.uniform(lo_t, hi_t)
            if saccade:
                t *= 0.6
            elif random.random() < long_prob:
                t *= random.uniform(1.8, 3.5)
            await asyncio.sleep(t)
