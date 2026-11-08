"""Messages between the ESP32 and the brain (smabo-brain or smabo-brain-ros).

Both brains serve ``ws://<brain>:9090/`` and take the same rosbridge v2.0 JSON
handshake, so nothing here depends on which one is running:

  on connect   advertise the topics below that we publish, subscribe the ones
               we read, then send the config snapshot.
  outbound     {"op":"publish","topic":"/esp32<topic>","msg":{...}}
                 /wheel_vel     {left, right (m/s), dt (s)}  → brain integrates /odom
                 /joint_states  sensor_msgs/JointState
                 /scan          sensor_msgs/LaserScan
                 /pong          std_msgs/String (echo of /ping)
                 /config        std_msgs/String, config JSON minus wifi
                 /config_url    std_msgs/String, our REST base URL ("http://ip:port"),
                                on connect and every 5 s, so smabo-web finds the
                                config API without being told the ESP32's address
                 /hello         std_msgs/String, {"protocol", "client", "version"}
                                once per connect (protocol check)
               {"op":"notice","message":"..."}
  inbound      {"op":"publish","topic":"/cmd_vel" | "/servo/command" | "/ping", ...}
               /brain/info  {"data": "{\"protocol\": N, ...}"}: warn if N differs

The ``/esp32`` source prefix lets the brain tell who sent a message; it strips
it and delivers the canonical name. These tables mirror smabo-brain's
brain/topics.py, whose tests check that they match.

Config and mode themselves are set over REST (http_server.py), not here.
"""

import json

# Protocol number of the topic tables below (smabo-brain's PROTOCOL_VERSION) and
# this firmware's version, sent on /hello after every connect.
PROTOCOL_VERSION = 1
CLIENT_NAME = "smabo-esp32"
FIRMWARE_VERSION = "0.1.0"

SOURCE_PREFIX = "/esp32"

ADVERTISE = (
    ("/esp32/config",       "std_msgs/msg/String"),
    ("/esp32/wheel_vel",    "smabo_interfaces/msg/WheelVel"),
    ("/esp32/joint_states", "sensor_msgs/msg/JointState"),
    ("/esp32/scan",         "sensor_msgs/msg/LaserScan"),
    ("/esp32/pong",         "std_msgs/msg/String"),
    ("/esp32/config_url",   "std_msgs/msg/String"),
    ("/esp32/hello",        "std_msgs/msg/String"),
)
SUBSCRIBE = (
    ("/cmd_vel",        "geometry_msgs/msg/Twist"),
    ("/servo/command",  "trajectory_msgs/msg/JointTrajectory"),
    ("/ping",           "std_msgs/msg/String"),
    ("/brain/info",     "std_msgs/msg/String"),
)


class BrainLink:
    """Formats outbound frames for, and parses inbound frames from, the brain.

    Parameters
    ----------
    ws : ws_client.WSClient
        The WebSocket client connected to the brain.
    """

    def __init__(self, ws):
        self.ws = ws

    def _send(self, frame):
        self.ws.broadcast(json.dumps(frame))

    def publish(self, topic, msg):
        """Publish ``msg`` on the canonical ``topic`` (the /esp32 prefix is added)."""
        self._send({"op": "publish", "topic": SOURCE_PREFIX + topic, "msg": msg})

    def notice(self, message):
        """Send a human-readable notice (shown in smabo-web)."""
        self._send({"op": "notice", "message": message})

    def send_config(self, config):
        """Send the config snapshot that smabo-brain needs for odometry.

        Wi-Fi credentials are left out: on smabo-brain-ros any rosbridge client
        could subscribe to this topic.
        """
        snapshot = {k: v for k, v in config.items() if k != "wifi"}
        self.publish("/config", {"data": json.dumps(snapshot)})

    def send_config_url(self, url):
        """Tell smabo-web where our REST config API is (``http://ip:port``)."""
        if url:
            self.publish("/config_url", {"data": url})

    def handshake(self, config, config_url=None):
        """Run on every (re)connect: advertise, subscribe, then send the config
        and the REST URL."""
        for topic, typ in ADVERTISE:
            self._send({"op": "advertise", "topic": topic, "type": typ})
        for topic, typ in SUBSCRIBE:
            self._send({"op": "subscribe", "topic": topic, "type": typ})
        self.send_config(config)
        self.send_config_url(config_url)
        self.send_hello()

    def send_hello(self):
        """Tell the brain which protocol and firmware version we speak."""
        self.publish("/hello", {"data": json.dumps({
            "protocol": PROTOCOL_VERSION, "client": CLIENT_NAME, "version": FIRMWARE_VERSION})})

    def check_brain_info(self, msg):
        """``/brain/info`` → warning text when the brain's protocol differs, else ``None``."""
        try:
            info = json.loads((msg or {}).get("data") or "{}")
        except ValueError:
            return None
        protocol = info.get("protocol") if isinstance(info, dict) else None
        if protocol is None or protocol == PROTOCOL_VERSION:
            return None
        return ("protocol mismatch: brain %s %s speaks %s, smabo-esp32 %s speaks %s"
                % (info.get("brain"), info.get("version"), protocol,
                   FIRMWARE_VERSION, PROTOCOL_VERSION))

    @staticmethod
    def parse(text):
        """Inbound frame → ``(topic, msg)`` for a publish, else ``None``."""
        try:
            data = json.loads(text)
        except ValueError:
            return None
        if data.get("op") != "publish":
            return None
        return data.get("topic"), data.get("msg") or {}
