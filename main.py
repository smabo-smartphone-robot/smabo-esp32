"""Entry point.

Boot order:
  1. load persistent config
  2. bring up WiFi (+ keepalive)
  3. init I2C + PCA9685 (servo PWM bus)
  4. start the WebSocket server (rosbridge-protocol)
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
from ws_server import WSServer
from robot import Robot
import wifi_manager


async def amain():
    """Bring up all subsystems and then idle forever.

    Returns
    -------
    None
        Runs for the program lifetime.
    """
    cfg = Config()

    sta = await wifi_manager.connect(cfg)

    # I2C bus + PCA9685 servo PWM controller (shared by all joints).
    # Wrapped so a missing board (OSError) or a GPIO that doesn't exist on
    # this chip (ValueError: invalid pin) disables servos instead of killing
    # boot. e.g. GPIO 21/22 exist on the classic ESP32 but not on the S3.
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

    # Robot needs the WS server to broadcast; WS server needs Robot's handler.
    robot_ref = {}

    def on_message(_client, text):
        """Forward an inbound WebSocket frame to the Robot handler.

        Parameters
        ----------
        _client : asyncio.StreamWriter
            The originating client (unused; broadcasts go to all clients).
        text : str
            The inbound frame's text payload.

        Returns
        -------
        None
        """
        return robot_ref["robot"].on_message(_client, text)

    ws = WSServer(cfg.get("ws.port", 9090), on_message)
    robot = Robot(cfg, pca, ws)
    robot_ref["robot"] = robot

    await ws.start()
    robot.start()

    # background services
    asyncio.create_task(cfg.autosave_task())
    asyncio.create_task(wifi_manager.keepalive_task(cfg, sta))

    print("Robot ready.")
    while True:
        await asyncio.sleep(3600)


def run():
    """Run the asyncio event loop until interrupted.

    Returns
    -------
    None
    """
    try:
        asyncio.run(amain())
    except KeyboardInterrupt:
        pass
    finally:
        asyncio.new_event_loop()


if __name__ == "__main__":
    run()
