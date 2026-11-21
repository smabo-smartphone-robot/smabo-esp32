"""Wheel mixing, the static-friction floor and the dead-man stop."""

import asyncio

import pytest

import dc_motors
from conftest import defaults_at
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
        return self.dc.get(path.split(".", 1)[1], defaults_at(path, default))


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


def test_stop_brakes_both_wheels_by_default(drive):
    drive.set_cmd_vel(0.3, 0.5)
    drive.stop()
    for m in (drive.left, drive.right):
        assert (m.in1.value(), m.in2.value()) == (1, 1)        # TB6612 short brake


def test_stop_coasts_when_brake_is_off():
    d = DiffDrive(Cfg(brake=False))
    d.set_cmd_vel(0.3, 0.5)
    d.stop()
    for m in (d.left, d.right):
        assert (m.in1.value(), m.in2.value(), m.pwm.duty()) == (0, 0, 0)


def test_zero_command_brakes(drive):
    drive.set_cmd_vel(0.2, 0.0)
    drive.set_cmd_vel(0.0, 0.0)
    assert (drive.left.in1.value(), drive.left.in2.value()) == (1, 1)


def test_kick_start_from_rest_then_commanded_duty(monkeypatch):
    now = [1000]
    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: now[0])
    d = DiffDrive(Cfg(start_duty=0.5, kick_ms=150))
    slow = 0.05                                   # well under the kick duty
    norm = 0.3 + 1.5 * 0.13 / 2
    d.set_cmd_vel(slow, 0.0)
    assert wheel(d.left) == wheel(d.right) == int(0.5 * 1023)      # kicked
    now[0] += 100
    d.set_cmd_vel(slow, 0.0)
    assert wheel(d.left) == int(0.5 * 1023)                         # still within the kick
    now[0] += 100
    d.set_cmd_vel(slow, 0.0)
    assert wheel(d.left) == int(slow / norm * 1023)                 # back to the command
    # reversing kicks again, the other way
    d.set_cmd_vel(-slow, 0.0)
    assert wheel(d.left) == -int(0.5 * 1023)
    # and so does starting after a stop
    now[0] += 500
    d.stop()
    d.set_cmd_vel(slow, 0.0)
    assert wheel(d.left) == int(0.5 * 1023)


def test_fast_commands_are_not_lowered_by_the_kick(monkeypatch):
    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: 0)
    d = DiffDrive(Cfg(start_duty=0.3, kick_ms=150))
    d.set_cmd_vel(0.3, 0.0)
    norm = 0.3 + 1.5 * 0.13 / 2
    assert wheel(d.left) == int(0.3 / norm * 1023)


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


# --------------------------------------------------------------------------- #
# wheel speed control (encoder_drive)
# --------------------------------------------------------------------------- #

@pytest.fixture
def pi():
    d = DiffDrive(Cfg(min_duty=0.7))       # the open-loop floor must not apply
    d.set_closed_loop(True)
    return d


def duty(motor):
    return wheel(motor) / 1023


def test_closed_loop_starts_from_the_feed_forward_duty(pi):
    pi.set_cmd_vel(0.0, 0.6)
    ff = 1 / (0.3 + 1.5 * 0.13 / 2)
    v = 0.6 * 0.13 / 2
    assert duty(pi.right) == pytest.approx(v * ff, abs=2e-3)
    assert duty(pi.left) == pytest.approx(-v * ff, abs=2e-3)


def test_slow_wheel_gets_more_duty_and_fast_wheel_less(pi):
    pi.set_cmd_vel(0.1, 0.0)
    start = duty(pi.left)
    pi.update_speed(0.0, 0.2, 0.05)        # left stalled, right too fast
    assert duty(pi.left) > start
    assert duty(pi.right) < start


def test_integrator_is_clamped_so_a_dead_encoder_cannot_run_away():
    d = DiffDrive(Cfg(speed_i_max=0.3))
    d.set_closed_loop(True)
    d.set_cmd_vel(0.05, 0.0)
    for _ in range(200):
        d.update_speed(0.0, 0.0, 0.05)     # encoder never counts
    ff = 0.05 / (0.3 + 1.5 * 0.13 / 2)
    assert duty(d.left) <= ff + 2.0 * 0.05 + 0.3 + 2e-3


def test_never_drives_against_the_target_nor_brakes_while_moving(pi, monkeypatch):
    now = [0]
    monkeypatch.setattr(dc_motors, "_ticks_ms", lambda: now[0])
    pi.cfg.dc.update(start_duty=0.55, kick_ms=120)
    pi.set_cmd_vel(0.05, 0.0)
    now[0] = 500                           # kick over
    pi.update_speed(1.0, 1.0, 0.05)        # far too fast
    for m in (pi.left, pi.right):
        assert (m.in1.value(), m.in2.value()) == (1, 0)    # forward, not braked
        assert 0 < duty(m) < 0.05


def test_zero_command_brakes(pi):
    pi.set_cmd_vel(0.1, 0.0)
    pi.update_speed(0.0, 0.0, 0.05)
    pi.set_cmd_vel(0.0, 0.0)
    for m in (pi.left, pi.right):
        assert (m.in1.value(), m.in2.value()) == (1, 1)
    pi.update_speed(0.0, 0.0, 0.05)        # stays braked
    assert (pi.left.in1.value(), pi.left.in2.value()) == (1, 1)


def test_speed_control_off_falls_back_to_open_loop():
    d = DiffDrive(Cfg(speed_control=False, min_duty=0.7))
    d.set_closed_loop(True)
    d.set_cmd_vel(0.0, 0.6)
    assert duty(d.right) == pytest.approx(0.7, abs=2e-3)


def test_open_loop_without_encoders_ignores_feedback():
    d = DiffDrive(Cfg())
    d.set_cmd_vel(0.1, 0.0)
    before = wheel(d.left)
    d.update_speed(0.0, 0.0, 0.05)
    assert wheel(d.left) == before


def test_converges_on_a_motor_that_stalls_below_half_duty(pi):
    """Stiction model like the real robot: no motion below 0.5 duty from rest,
    keeps turning down to 0.3 once moving; speed = 0.25 m/s * duty."""
    target = 0.6 * 0.13 / 2                 # 0.6 rad/s spin
    pi.set_cmd_vel(0.0, 0.6)
    moving = False
    speeds = []
    for _ in range(200):                    # 10 s at 20 Hz
        u = duty(pi.right)
        moving = u >= (0.3 if moving else 0.5)
        v = 0.25 * u if moving else 0.0
        speeds.append(v)
        pi.update_speed(-v, v, 0.05)
    mean = sum(speeds[100:]) / 100          # last 5 s
    assert mean == pytest.approx(target, rel=0.15)


def test_speed_control_does_not_kick_start(pi):
    pi.cfg.dc.update(start_duty=0.55, kick_ms=120)
    pi.set_cmd_vel(0.0, 0.6)
    assert duty(pi.right) < 0.2


def test_next_start_reuses_the_friction_learned_on_the_last_run(pi):
    pi.set_cmd_vel(0.0, 0.6)
    for _ in range(40):
        pi.update_speed(0.0, 0.0, 0.05)    # stalled: integrator climbs
    needed = pi._integ[1]
    assert needed > 0.3
    pi.set_cmd_vel(0.0, 0.0)               # stop
    pi.set_cmd_vel(0.0, -0.6)              # start the other way
    ff = 1 / (0.3 + 1.5 * 0.13 / 2)
    assert duty(pi.right) == pytest.approx(-(0.039 * ff + needed), abs=2e-3)


def test_integrator_never_cancels_more_than_the_feed_forward(pi):
    pi.set_cmd_vel(0.0, 0.6)
    for _ in range(100):
        pi.update_speed(-0.5, 0.5, 0.05)   # much too fast for a long time
    ff = 1 / (0.3 + 1.5 * 0.13 / 2)
    v = 0.6 * 0.13 / 2
    assert pi._integ[1] == pytest.approx(-v * ff)
    assert pi._integ[0] == pytest.approx(v * ff)


def test_on_target_wheel_keeps_the_feed_forward_duty(pi):
    pi.set_cmd_vel(0.0, 0.6)
    v = 0.6 * 0.13 / 2
    ff = 1 / (0.3 + 1.5 * 0.13 / 2)
    pi.update_speed(-v, v, 0.05)           # both wheels exactly on target
    assert duty(pi.right) == pytest.approx(v * ff, abs=2e-3)
    assert duty(pi.left) == pytest.approx(-v * ff, abs=2e-3)
