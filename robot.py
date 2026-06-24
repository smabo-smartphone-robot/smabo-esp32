"""Orchestrator: wires subsystems together, handles the rosbridge protocol,
and manages which modes are active.

WebSocket message protocol (rosbridge v2.0 compatible JSON), via smabo-brain:

  inbound — robot operation (publish):
    {"op":"publish","topic":"/cmd_vel",       "msg":{geometry_msgs/Twist}}
    {"op":"publish","topic":"/servo/command", "msg":{trajectory_msgs/JointTrajectory}}

  outbound:
    {"op":"publish","topic":"/wheel_vel",    "msg":{left, right (m/s), dt (s)}}
    {"op":"publish","topic":"/joint_states", "msg":{sensor_msgs/JointState}}
    {"op":"set_config","config":{...}}       full config snapshot for brain's
                                             odometry sync (see below)
    {"op":"notice", "message":"..."}

  Note: raw wheel velocities are published; smabo-brain integrates them
  into nav_msgs/Odometry (so it can be fused with IMU/GPS).

Config / mode are NOT handled over WebSocket — smabo-web sets them directly
via the REST API in http_server.py (GET/POST /config, POST /mode).  After a
change (and on each brain reconnect) the full config is pushed to smabo-brain
over WebSocket so its odometry integrator stays in sync with the wheel
geometry / covariance / frame names.
"""

import json

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

try:
    import time
    _ticks_ms = time.ticks_ms
except AttributeError:
    import time
    _ticks_ms = lambda: int(time.time() * 1000)

from servo_controller import JointGroup
from random_motion import RandomMotion
from dc_motors import DiffDrive
from wheel_publisher import WheelPublisher
from lidar_ld06 import Ld06

# Topics advertised / subscribed when talking to smabo-brain-ros (rosbridge),
# which—unlike the legacy relay—requires explicit advertise before publish and
# subscribe to receive. Outbound carry the /esp32 source prefix (stripped by the
# relay); inbound are canonical names.
_ROSBRIDGE_ADVERTISE = (
    ("/esp32/wheel_vel",    "smabo_interfaces/msg/WheelVel"),
    ("/esp32/joint_states", "sensor_msgs/msg/JointState"),
    ("/esp32/scan",         "sensor_msgs/msg/LaserScan"),
    ("/esp32/pong",         "std_msgs/msg/String"),   # WS ping echo (see _on_publish)
)
_ROSBRIDGE_SUBSCRIBE = (
    ("/cmd_vel",        "geometry_msgs/msg/Twist"),
    ("/servo/command",  "trajectory_msgs/msg/JointTrajectory"),
    ("/ping",           "std_msgs/msg/String"),       # WS ping request
)


def _reboot_reasons(config_dict):
    """Find config keys in a patch that require a full hardware reboot.

    Parameters
    ----------
    config_dict : dict
        A ``set_config`` patch (nested overrides).

    Returns
    -------
    list of str
        Dot-paths needing a reboot (e.g. ``"dc.pins"``); empty if none.
    """
    reasons = []
    for key in ("i2c", "pca9685", "wifi"):
        if key in config_dict:
            reasons.append(key)
    dc = config_dict.get("dc")
    if isinstance(dc, dict) and "pins" in dc:
        reasons.append("dc.pins")
    enc = config_dict.get("encoder")
    if isinstance(enc, dict):
        for side in ("left", "right"):
            if side in enc:
                reasons.append("encoder.%s" % side)
    return reasons


class Robot:
    """Top-level orchestrator: owns subsystems, tasks, and message dispatch."""

    def __init__(self, config, pca, ws):
        """Store dependencies and initialise empty subsystem/task state.

        Parameters
        ----------
        config : config.Config
            The shared configuration instance.
        pca : pca9685.PCA9685
            The servo PWM controller shared by all joints.
        ws : ws_client.WSClient
            The WebSocket client to smabo-brain, used to send outbound messages.
        """
        self.cfg = config
        self.pca = pca
        self.ws = ws

        self.servo_group   = None   # JointGroup for all servos
        self.random_motion = None   # RandomMotion instance
        self.drive         = None   # DiffDrive
        self.encoders      = None
        self.wheel_pub     = None
        self.lidar         = None   # Ld06 → /scan

        self._tasks = {}

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def publish(self, topic, msg):
        """Broadcast a rosbridge ``publish`` op to all clients.

        Parameters
        ----------
        topic : str
            ROS topic name.
        msg : dict
            The message payload matching the topic's type.

        Returns
        -------
        None

        Notes
        -----
        The source prefix ``/esp32`` is prepended so smabo-brain can identify
        the origin and strip it before re-broadcasting under the canonical
        topic name.
        """
        self.ws.broadcast(json.dumps({"op": "publish", "topic": "/esp32" + topic, "msg": msg}))

    def _notice(self, message):
        """Broadcast a ``notice`` op (informational message) to all clients.

        Parameters
        ----------
        message : str
            Human-readable notice text.

        Returns
        -------
        None
        """
        self.ws.broadcast(json.dumps({"op": "notice", "message": message}))

    def sync_config_to_brain(self):
        """Push the full config to smabo-brain so its odometry stays in sync.

        Config is otherwise set directly by smabo-web over REST (and never
        flows through brain), but brain's odometry integrator needs the wheel
        geometry / covariance / frame names.  Called on every brain (re)connect
        and after each REST config/mode change.

        Returns
        -------
        None
        """
        # rosbridge (smabo-brain-ros): odometry params come from the ROS launch,
        # not from the ESP32, and ``set_config`` is not a rosbridge op — skip it.
        if self.cfg.get("brain.rosbridge"):
            return
        self.ws.broadcast(json.dumps({"op": "set_config", "config": self.cfg.data}))

    def on_brain_connect(self):
        """Run once on every (re)connect to the brain/rosbridge endpoint.

        Legacy relay: push the config snapshot for odometry sync. rosbridge:
        advertise outbound topics and subscribe to inbound ones (required before
        publish/receive). Called from the WSClient ``on_connect`` hook.

        Returns
        -------
        None
        """
        if self.cfg.get("brain.rosbridge"):
            for topic, typ in _ROSBRIDGE_ADVERTISE:
                self.ws.broadcast(json.dumps({"op": "advertise", "topic": topic, "type": typ}))
            for topic, typ in _ROSBRIDGE_SUBSCRIBE:
                self.ws.broadcast(json.dumps({"op": "subscribe", "topic": topic, "type": typ}))
        else:
            self.sync_config_to_brain()

    # ------------------------------------------------------------------ #
    # config / mode REST API (called by http_server.ConfigHTTPServer)
    # ------------------------------------------------------------------ #
    def config_json(self):
        """Return the full live config dict (for ``GET /config``).

        Returns
        -------
        dict
            The current configuration.
        """
        return self.cfg.data

    def set_config(self, patch):
        """Apply a config patch from REST, then resync brain (``POST /config``).

        Parameters
        ----------
        patch : dict
            A ``set_config`` patch (nested overrides).

        Returns
        -------
        None
        """
        self._on_set_config(patch or {})
        self.sync_config_to_brain()

    def set_mode(self, partial):
        """Apply a partial mode update from REST, then resync brain (``POST /mode``).

        Parameters
        ----------
        partial : dict
            Mode flags to override (unspecified flags keep their value).

        Returns
        -------
        None
        """
        self.apply_modes(self._merged_modes(partial or {}))
        self.sync_config_to_brain()

    # ------------------------------------------------------------------ #
    # task lifecycle
    # ------------------------------------------------------------------ #
    def _spawn(self, name, coro):
        """Start a coroutine as a named task, ignoring duplicates.

        Parameters
        ----------
        name : str
            Unique task name; if already present, ``coro`` is not started.
        coro : coroutine
            The coroutine to schedule.

        Returns
        -------
        None
        """
        if name in self._tasks:
            return
        self._tasks[name] = asyncio.create_task(coro)

    def _kill(self, name):
        """Cancel and forget the named task if it exists.

        Parameters
        ----------
        name : str
            The task name to cancel.

        Returns
        -------
        None
        """
        t = self._tasks.pop(name, None)
        if t is not None:
            try:
                t.cancel()
            except Exception:
                pass

    def _kill_prefix(self, prefix):
        """Cancel every task whose name starts with a prefix.

        Parameters
        ----------
        prefix : str
            The task-name prefix to match (e.g. ``"random_"``).

        Returns
        -------
        None
        """
        for name in [k for k in self._tasks if k.startswith(prefix)]:
            self._kill(name)

    # ------------------------------------------------------------------ #
    # servo subsystem (all PCA9685 joints: head, hands, arm, …)
    # ------------------------------------------------------------------ #
    def _teardown_servos(self):
        """Cancel all servo tasks and drop the JointGroup / RandomMotion.

        Returns
        -------
        None
        """
        self._kill("servo_follower")
        self._kill("joint_states")
        self._kill_prefix("random_")   # one task per group: random_<name>
        self.servo_group   = None
        self.random_motion = None

    def enable_servos(self, on):
        """Start or tear down the servo subsystem.

        When enabled, (re)creates the JointGroup if needed and spawns the
        follower, one task per random group, and the joint_states publisher.

        Parameters
        ----------
        on : bool
            True to start the subsystem, False to tear it down.

        Returns
        -------
        None
        """
        if on:
            if self.pca is None:
                print("サーボ: PCA9685未接続のためスキップします")
                return
            if self.servo_group is None:
                self.servo_group   = JointGroup(self.pca, self.cfg.get("servos.joints"))
                self.random_motion = RandomMotion(self.servo_group, self.cfg)
            self._spawn("servo_follower", self.servo_group.run())
            # one independent task per random group
            for g in (self.cfg.get("servos.random_groups") or []):
                gname = g.get("name", "")
                self._spawn("random_%s" % gname,
                            self.random_motion.run_group(gname))
            self._spawn("joint_states", self._joint_states_task())
        else:
            self._teardown_servos()

    # ------------------------------------------------------------------ #
    # drive subsystem
    # ------------------------------------------------------------------ #
    def _teardown_drive(self):
        """Cancel drive/odom tasks, stop the motors and drop all drive objects.

        Returns
        -------
        None
        """
        self._kill("drive_wd")
        self._kill("odom")
        if self.drive is not None:
            self.drive.stop()
        self.drive    = None
        self.encoders  = None
        self.wheel_pub = None

    def enable_drive(self, with_encoder):
        """Start the differential drive, optionally with wheel feedback.

        Creates the DiffDrive (and, if requested, the encoders/wheel publisher)
        on first use and spawns the watchdog (and wheel_vel publisher) tasks.

        Parameters
        ----------
        with_encoder : bool
            If True, also create encoders and publish ``/wheel_vel``.

        Returns
        -------
        None
        """
        if self.drive is None:
            self.drive = DiffDrive(self.cfg)
        self._spawn("drive_wd", self.drive.watchdog_task())
        if with_encoder:
            from encoder import QuadEncoder
            if self.encoders is None:
                el = self.cfg.get("encoder.left")
                er = self.cfg.get("encoder.right")
                self.encoders  = (QuadEncoder(el["a"], el["b"]),
                                  QuadEncoder(er["a"], er["b"]))
                self.wheel_pub = WheelPublisher(self.cfg, self.encoders[0], self.encoders[1])
            self._spawn("odom", self.wheel_pub.run(self.publish))

    def disable_drive(self):
        """Stop drive tasks and motors but keep the DiffDrive object for reuse.

        Returns
        -------
        None
        """
        self._kill("drive_wd")
        self._kill("odom")
        if self.drive is not None:
            self.drive.stop()

    # ------------------------------------------------------------------ #
    # lidar subsystem (LD06 → /scan)
    # ------------------------------------------------------------------ #
    def _teardown_lidar(self):
        """Cancel the scan task and release the LD06 UART."""
        self._kill("scan")
        if self.lidar is not None:
            self.lidar.deinit()
        self.lidar = None

    def enable_lidar(self, on):
        """Start or tear down the LD06 /scan publisher.

        Parameters
        ----------
        on : bool
            True to open the UART and publish ``/scan``, False to tear down.
        """
        if on:
            if self.lidar is None:
                self.lidar = Ld06(self.cfg)
            self._spawn("scan", self.lidar.run(self.publish))
        else:
            self._teardown_lidar()

    # ------------------------------------------------------------------ #
    # mode management
    # ------------------------------------------------------------------ #
    def apply_modes(self, modes):
        """Enable/disable subsystems to match ``modes`` and persist the result.

        Enforces the dc_drive / encoder_drive exclusivity (encoder wins) and
        writes the resolved modes back to config.

        Parameters
        ----------
        modes : dict
            Desired mode flags (``servos``, ``dc_drive``, ``encoder_drive``);
            mutated in place to apply the exclusivity rule.

        Returns
        -------
        None
        """
        if modes.get("encoder_drive") and modes.get("dc_drive"):
            modes["dc_drive"] = False

        self.enable_servos(modes.get("servos", False))
        self.enable_lidar(modes.get("lidar", False))

        if modes.get("encoder_drive"):
            self.disable_drive()
            self.enable_drive(with_encoder=True)
        elif modes.get("dc_drive"):
            self._kill("odom")
            self.enable_drive(with_encoder=False)
        else:
            self.disable_drive()

        self.cfg.update({"modes": modes})

    def start(self):
        """Apply the persisted modes once at boot.

        Returns
        -------
        None
        """
        self.apply_modes(dict(self.cfg.get("modes")))

    # ------------------------------------------------------------------ #
    # config hot-reload
    # ------------------------------------------------------------------ #
    def _reload_subsystems(self, changed_keys):
        """Restart the subsystems affected by changed top-level config keys.

        Only restarts a subsystem if its corresponding mode is active.

        Parameters
        ----------
        changed_keys : list of str
            Top-level config keys that changed (e.g. ``"servos"``, ``"dc"``).

        Returns
        -------
        None
        """
        modes = self.cfg.get("modes")
        for key in changed_keys:
            if key == "servos":
                if modes.get("servos"):
                    self._teardown_servos()
                    self.enable_servos(True)

            elif key == "dc":
                if modes.get("dc_drive") or modes.get("encoder_drive"):
                    self._teardown_drive()
                    self.enable_drive(with_encoder=bool(modes.get("encoder_drive")))

            elif key == "encoder":
                if modes.get("encoder_drive"):
                    self._teardown_drive()
                    self.enable_drive(with_encoder=True)

            elif key == "lidar":
                if modes.get("lidar"):
                    self._teardown_lidar()
                    self.enable_lidar(True)

    # ------------------------------------------------------------------ #
    # /joint_states publisher (required by MoveIt2)
    # ------------------------------------------------------------------ #
    async def _joint_states_task(self):
        """Publish sensor_msgs/JointState for all servos at the configured rate.

        Idle-polls when the rate is 0 or the servo subsystem is absent.

        Returns
        -------
        None
            Never returns (cancelled when the servo subsystem restarts).
        """
        while True:
            rate = self.cfg.get("servos.joint_states_rate", 20.0)
            if rate > 0 and self.servo_group is not None:
                state  = self.servo_group.get_state()   # {name: rad}
                now_ms = _ticks_ms()
                stamp  = {"sec": now_ms // 1000,
                           "nanosec": (now_ms % 1000) * 1_000_000}
                self.publish("/joint_states", {
                    "header":   {"stamp": stamp, "frame_id": ""},
                    "name":     list(state.keys()),
                    "position": list(state.values()),
                    "velocity": [0.0] * len(state),
                    "effort":   [0.0] * len(state),
                })
            await asyncio.sleep(1.0 / rate if rate > 0 else 0.05)

    # ------------------------------------------------------------------ #
    # reboot helper
    # ------------------------------------------------------------------ #
    async def _reboot_after(self, ms, reasons):
        """Notify clients, wait so the frame flushes, then hard-reset the MCU.

        Parameters
        ----------
        ms : int
            Delay in milliseconds before resetting (lets the notice flush).
        reasons : list of str
            Config dot-paths that triggered the reboot (for the notice text).

        Returns
        -------
        None
            Does not return on hardware (the board resets).
        """
        self._notice("Rebooting to apply pin/hw changes: " + ", ".join(reasons))
        await asyncio.sleep_ms(ms)
        try:
            import machine
            machine.reset()
        except ImportError:
            print("(off-target) would reboot now:", reasons)

    # ------------------------------------------------------------------ #
    # inbound message handling
    # ------------------------------------------------------------------ #
    def on_message(self, _client, text):
        """Parse one inbound WebSocket frame and dispatch it by its ``op`` field.

        Only real-time ``publish`` ops are handled here; config and mode are
        set out-of-band via the REST API (see http_server.py).  Invalid JSON
        and unknown ops are ignored.

        Parameters
        ----------
        _client : asyncio.StreamWriter
            The originating client (unused; replies are broadcast).
        text : str
            The raw JSON text of the inbound frame.

        Returns
        -------
        None
        """
        try:
            data = json.loads(text)
        except ValueError:
            return
        op = data.get("op")
        if op == "publish":
            self._on_publish(data.get("topic"), data.get("msg") or {})
        elif op in ("subscribe", "advertise", "unsubscribe"):
            pass

    def _merged_modes(self, partial):
        """Overlay a partial mode update onto the current modes.

        Parameters
        ----------
        partial : dict
            Mode flags to override (unspecified flags keep their value).

        Returns
        -------
        dict
            A new dict of the merged mode flags.
        """
        modes = dict(self.cfg.get("modes"))
        modes.update(partial)
        return modes

    def _on_set_config(self, config_dict):
        """Persist a config patch, then reboot or hot-reload as appropriate.

        Pin/bus changes save immediately and schedule a reboot; everything
        else triggers a targeted subsystem reload.

        Parameters
        ----------
        config_dict : dict
            The ``set_config`` patch (nested overrides).

        Returns
        -------
        None
        """
        reasons = _reboot_reasons(config_dict)

        self.cfg.update(config_dict)
        if reasons:
            self.cfg.save_now()
            asyncio.create_task(self._reboot_after(500, reasons))
        else:
            self._reload_subsystems(list(config_dict.keys()))

    def _on_publish(self, topic, msg):
        """Route an inbound ``publish`` to the drive or servo subsystem by topic.

        Parameters
        ----------
        topic : str
            The published topic (``/cmd_vel``, ``/servo/command`` or ``/ping``).
        msg : dict
            The ROS message payload for that topic.

        Returns
        -------
        None
        """
        if topic == "/cmd_vel":
            if self.drive is not None:
                lin = (msg.get("linear")  or {}).get("x", 0.0)
                ang = (msg.get("angular") or {}).get("z", 0.0)
                self.drive.set_cmd_vel(lin, ang)

        elif topic == "/servo/command":
            names  = msg.get("joint_names") or []
            points = msg.get("points") or []
            if names and points and self.servo_group is not None:
                self.servo_group.load_trajectory(names, points)

        elif topic == "/ping":
            # Application-level echo so smabo-web can measure end-to-end WS
            # reachability (web → brain → ESP32 → brain → web). Mirror the
            # payload back on /pong unchanged; publish() adds the /esp32 prefix.
            # std_msgs/String shape: {"data": "<token>"} (an opaque token the
            # web side matches to compute the round-trip time).
            self.publish("/pong", {"data": (msg or {}).get("data", "")})
