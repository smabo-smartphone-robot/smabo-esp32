"""Pure helpers in robot.py: masked Wi-Fi passwords and reboot reasons."""

from robot import WIFI_PASSWORD_MASK, _reboot_reasons, drop_masked_wifi_password


def test_masked_password_is_dropped_and_other_wifi_keys_kept():
    patch = {"wifi": {"ssid": "home", "password": WIFI_PASSWORD_MASK}, "dc": {"max_linear": 0.2}}
    assert drop_masked_wifi_password(patch) == {"wifi": {"ssid": "home"}, "dc": {"max_linear": 0.2}}
    assert patch["wifi"]["password"] == WIFI_PASSWORD_MASK  # input untouched


def test_wifi_with_only_the_mask_disappears():
    assert drop_masked_wifi_password({"wifi": {"password": WIFI_PASSWORD_MASK}}) == {}


def test_a_real_password_goes_through():
    patch = {"wifi": {"password": "secret"}}
    assert drop_masked_wifi_password(patch) == patch


def test_reboot_reasons():
    assert _reboot_reasons({"dc": {"max_linear": 0.2}}) == []
    assert _reboot_reasons({"wifi": {}, "dc": {"pins": {}}, "encoder": {"left": {}, "x": 1}}) == [
        "wifi", "dc.pins", "encoder.left"]


# ---------------------------------------------------------------------------
# apply_modes: the drive restarts only when its mode changes
# ---------------------------------------------------------------------------
import asyncio  # noqa: E402

from config import Config  # noqa: E402
from robot import Robot  # noqa: E402


class _FakeWs:
    def send(self, *_a, **_k):
        pass


def _robot(tmp_path):
    return Robot(Config(str(tmp_path / "config.json")), pca=None, ws=_FakeWs())


def test_toggling_the_lidar_keeps_the_drive_running(tmp_path):
    async def run():
        r = _robot(tmp_path)
        r.apply_modes({"encoder_drive": True})
        drive, wd, odom = r.drive, r._tasks["drive_wd"], r._tasks["odom"]
        r.drive.set_cmd_vel(0.1, 0.0)
        target = r.drive._target
        r.apply_modes({"encoder_drive": True, "servos": True})    # pca=None: servos skip
        assert r.drive is drive and r._tasks["drive_wd"] is wd and r._tasks["odom"] is odom
        assert r.drive._target == target                           # not stopped
        r.apply_modes({})
    asyncio.run(run())


def test_switching_drive_mode_restarts_the_drive(tmp_path):
    async def run():
        r = _robot(tmp_path)
        r.apply_modes({"encoder_drive": True})
        assert r.drive.closed_loop and "odom" in r._tasks
        r.apply_modes({"dc_drive": True})
        assert not r.drive.closed_loop and "odom" not in r._tasks and "drive_wd" in r._tasks
        r.apply_modes({})
        assert "drive_wd" not in r._tasks
        r.apply_modes({"encoder_drive": True, "dc_drive": True})  # encoder wins
        assert r.drive.closed_loop and "odom" in r._tasks
        r.apply_modes({})
    asyncio.run(run())
