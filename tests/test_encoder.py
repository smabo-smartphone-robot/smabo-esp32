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
