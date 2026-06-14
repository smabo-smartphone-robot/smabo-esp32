"""Entry point.

Boot order:
  1. load persistent config
  2. bring up WiFi (+ keepalive)
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
from robot import Robot
import wifi_manager


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

    ws = WSClient(
        host=cfg.get("brain.host", "192.168.1.100"),
        port=cfg.get("brain.port", 9090),
        path="/esp32",
        on_message=on_message,
    )
    robot = Robot(cfg, pca, ws)
    robot_ref["robot"] = robot

    asyncio.create_task(ws.run())
    robot.start()

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
