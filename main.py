"""Entry point.

Boot order:
  1. load persistent config
  2. bring up WiFi (+ keepalive); if it fails, offer Wi-Fi setup over BLE
  3. init I2C + PCA9685 (servo PWM bus)
  4. connect to smabo-brain as a WebSocket client
  5. build the Robot orchestrator and enable the configured modes
  6. run the asyncio event loop forever
"""

from machine import I2C, Pin

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

from config import Config
from pca9685 import PCA9685
from ws_client import WSClient
from http_server import ConfigHTTPServer
from robot import Robot
from pairing import Pairing
import wifi_manager
import improv


async def amain():
    cfg = Config()

    sta = await wifi_manager.connect(cfg)

    # I2C bus + PCA9685 servo PWM controller (shared by all joints).
    try:
        i2c = I2C(0,
                  scl=Pin(cfg.get("i2c.scl", 22)),
                  sda=Pin(cfg.get("i2c.sda", 21)),
                  freq=cfg.get("i2c.freq", 400000))
        pca = PCA9685(i2c,
                      address=cfg.get("pca9685.address", 0x40),
                      freq=cfg.get("pca9685.freq", 50))
    except (OSError, ValueError) as e:
        print("PCA9685: 未接続のためサーボ機能を無効化します ({})".format(e))
        pca = None

    robot_ref = {}

    def on_message(client, text):
        return robot_ref["robot"].on_message(client, text)

    def on_connect():
        # advertise outbound + subscribe inbound topics, then push config for
        # brain odometry. (config itself is set over REST.)
        return robot_ref["robot"].on_brain_connect()

    # smabo-brain and smabo-brain-ros both serve the rosbridge handshake at "/";
    # brain.path overrides.
    ws_path = cfg.get("brain.path") or "/"
    pairing = Pairing(cfg, name=cfg.get("wifi.hostname") or "smabo-esp32")
    ws = WSClient(
        host=cfg.get("brain.host", "192.168.1.100"),
        port=cfg.get("brain.port", 9090),
        path=ws_path,
        on_message=on_message,
        on_connect=on_connect,
        pairing=pairing,
    )
    robot = Robot(cfg, pca, ws, pairing)
    robot_ref["robot"] = robot

    # Wi-Fi down → let smabo-web send new credentials (and the brain's address
    # with an invite) over Bluetooth (Improv)
    def brain_moved(host, port):
        ws.host, ws.port = host, port
        ws.restart()
    improv.start(cfg, sta, brain_moved)

    asyncio.create_task(ws.run())
    asyncio.create_task(ConfigHTTPServer(robot, cfg.get("http.port", 80), pairing).run())
    robot.start()
    asyncio.create_task(robot.announce_config_url_task())

    asyncio.create_task(cfg.autosave_task())
    asyncio.create_task(wifi_manager.keepalive_task(cfg, sta))

    print("Robot ready. Connecting to brain at %s:%d …" % (
        cfg.get("brain.host"), cfg.get("brain.port")))
    while True:
        await asyncio.sleep(3600)


def run():
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass
    finally:
        asyncio.new_event_loop()


if __name__ == "__main__":
    run()
