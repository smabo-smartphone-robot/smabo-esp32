"""Wheel mixing, the static-friction floor and the dead-man stop."""

import asyncio

import pytest

import dc_motors
from dc_motors import DiffDrive, _with_min_duty


class Cfg:
    """Just enough of config.Config for DiffDrive."""

    def __init__(self, **dc):
        self.dc = {
            "pins": {"stby": 1, "ain1": 2, "ain2": 3, "pwma": 4, "bin1": 5, "bin2": 6, "pwmb": 7},
            "pwm_freq": 1000, "max_linear": 0.3, "max_angular": 1.5,
            "wheel_separation": 0.13, "min_duty": 0.0, "cmd_timeout": 0.5,
        }
        self.dc.update(dc)

    def get(self, path, default=None):
        return self.dc.get(path.split(".", 1)[1], default)


def wheel(motor):
    """Signed duty a motor is driving: +forward, -backward, 0 coast."""
    sign = motor.in1.value() - motor.in2.value()
    return sign * motor.pwm.duty()


@pytest.fixture
def drive():
    return DiffDrive(Cfg())


def test_forward_drives_both_wheels_equally(drive):
    drive.set_cmd_vel(0.3, 0.0)
    norm = 0.3 + 1.5 * 0.13 / 2
    expected = int(0.3 / norm * 1023)
    assert wheel(drive.left) == expected
    assert wheel(drive.right) == expected


def test_spin_counter_clockwise_turns_wheels_opposite(drive):
    drive.set_cmd_vel(0.0, 1.0)
    assert wheel(drive.left) < 0 < wheel(drive.right)
    assert wheel(drive.left) == -wheel(drive.right)


def test_commands_beyond_the_limits_clamp_to_full_duty(drive):
    drive.set_cmd_vel(5.0, 0.0)
    assert wheel(drive.left) == wheel(drive.right) == 1023


def test_inverted_motor_flips_direction():
    d = DiffDrive(Cfg(invert_left=True))
    d.set_cmd_vel(0.2, 0.0)
    assert wheel(d.left) == -wheel(d.right) < 0


def test_min_duty_lifts_small_commands_and_keeps_zero():
    assert _with_min_duty(0.05, 0.3) == 0.3
    assert _with_min_duty(-0.05, 0.3) == -0.3
    assert _with_min_duty(0.5, 0.3) == 0.5
    assert _with_min_duty(0.0, 0.3) == 0.0
    assert _with_min_duty(0.05, 0.0) == 0.05


def test_stop_coasts_both_wheels(drive):
    drive.set_cmd_vel(0.3, 0.5)
    drive.stop()
    for m in (drive.left, drive.right):
        assert (m.in1.value(), m.in2.value(), m.pwm.duty()) == (0, 0, 0)


class _OneTick(Exception):
    pass


def _run_one_watchdog_tick(drive, monkeypatch, now_ms):
    async def sleep_ms(_ms):
        raise _OneTick

    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: now_ms)
    monkeypatch.setattr(dc_motors, "asyncio", type("A", (), {"sleep_ms": staticmethod(sleep_ms)}))
    with pytest.raises(_OneTick):
        asyncio.run(drive.watchdog_task())


def test_watchdog_stops_when_commands_stop_arriving(drive, monkeypatch):
    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: 1000)
    drive.set_cmd_vel(0.3, 0.0)
    _run_one_watchdog_tick(drive, monkeypatch, now_ms=1000 + 501)
    assert wheel(drive.left) == wheel(drive.right) == 0


def test_watchdog_keeps_driving_within_the_timeout(drive, monkeypatch):
    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: 1000)
    drive.set_cmd_vel(0.3, 0.0)
    _run_one_watchdog_tick(drive, monkeypatch, now_ms=1000 + 400)
    assert wheel(drive.left) > 0
