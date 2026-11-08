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
