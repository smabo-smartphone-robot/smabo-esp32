import encoder
from encoder import QuadEncoder


def test_pins_use_the_internal_pull_up():
    enc = QuadEncoder(19, 21)
    assert (enc._a.num, enc._a.pull) == (19, encoder.Pin.PULL_UP)
    assert (enc._b.num, enc._b.pull) == (21, encoder.Pin.PULL_UP)


def test_falls_back_to_plain_input_where_the_pin_has_no_pull(monkeypatch):
    real = encoder.Pin

    class NoPull(real):
        def __init__(self, num, mode=None, pull=None):
            if pull is not None:
                raise ValueError("invalid pull")
            super().__init__(num, mode)

    monkeypatch.setattr(encoder, "Pin", NoPull)
    enc = QuadEncoder(34, 35)
    assert enc._a.pull is None and enc._b.pull is None


def _turn(enc, states):
    """Drive the fake pins through A/B states, firing the edge handler each time."""
    for a, b in states:
        enc._a.value(a)
        enc._b.value(b)
        enc._on_edge(None)


FORWARD = [(0, 1), (0, 0), (1, 0), (1, 1)]   # from 11: one full cycle, 4 steps


def test_counts_four_steps_per_cycle_forward_and_back():
    enc = QuadEncoder(41, 42)
    enc._a.value(1); enc._b.value(1); enc._prev = 3
    _turn(enc, FORWARD * 2)
    assert enc.read_and_reset() == 8
    _turn(enc, list(reversed(FORWARD[:-1])) + [(1, 1)])
    assert enc.read_and_reset() == -4


def test_ignores_bounces_and_impossible_jumps():
    enc = QuadEncoder(41, 42)
    enc._a.value(1); enc._b.value(1); enc._prev = 3
    _turn(enc, [(1, 1), (0, 0), (1, 1)])     # no change, then both channels at once
    assert enc.read_and_reset() == 0


def test_one_dead_channel_counts_nothing():
    # B stuck low (a broken or unpowered channel): A alone gives +1 / -1 in turn
    enc = QuadEncoder(41, 42)
    enc._a.value(0); enc._b.value(0); enc._prev = 0
    _turn(enc, [(1, 0), (0, 0)] * 10)
    assert enc.read_and_reset() == 0
