"""Quadrature encoder counting using GPIO interrupts.

NOTE: This is a software (IRQ) decoder and *will* lose counts at high speed
or high resolution.  For reliable counting use a MicroPython build that
exposes the ESP32 PCNT peripheral and swap this class out.

The A/B pins use the internal pull-up, so open-collector encoders work
without external resistors (push-pull encoders are unaffected). Classic ESP32
GPIO 34-39 have NO internal pull resistors: encoders wired there need external
pull-ups.
"""

from machine import Pin


def _input(pin):
    """Return an input Pin with the internal pull-up, or without where the pin has none."""
    try:
        return Pin(pin, Pin.IN, Pin.PULL_UP)
    except (ValueError, OSError):
        return Pin(pin, Pin.IN)


# Count step for (previous AB << 2) | current AB, AB = (A << 1) | B.
# Forward: 11→01, 01→00, 00→10, 10→11 (+1); the reverse order counts -1.
# No change, or both channels changing at once, counts 0.
_STEP = (0, -1, 1, 0,
         1, 0, 0, -1,
         -1, 0, 0, 1,
         0, 1, -1, 0)


class QuadEncoder:
    """Software x4 quadrature decoder driven by GPIO edge interrupts (state table)."""

    def __init__(self, pin_a, pin_b):
        """Attach edge interrupt handlers to the A and B channel pins.

        Parameters
        ----------
        pin_a : int
            GPIO number of the encoder A channel.
        pin_b : int
            GPIO number of the encoder B channel.
        """
        self._count = 0
        self._a = _input(pin_a)
        self._b = _input(pin_b)
        self._prev = self._state()
        self._a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_edge)
        self._b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_edge)

    def _state(self):
        """Return the current A/B levels packed as ``(A << 1) | B``."""
        return (self._a.value() << 1) | self._b.value()

    def _on_edge(self, _pin):
        """Handle an edge on either channel: step the count from the state change.

        The previous and current A/B states index ``_STEP``; a change that is
        not one quadrature step (noise, contact bounce, a missed edge) counts 0.

        Parameters
        ----------
        _pin : machine.Pin
            The triggering pin (supplied by the IRQ, unused).

        Returns
        -------
        None
        """
        cur = self._state()
        self._count += _STEP[(self._prev << 2) | cur]
        self._prev = cur

    def read_and_reset(self):
        """Return the count accumulated since the last call and zero it.

        Returns
        -------
        int
            Net signed edge count since the previous ``read_and_reset``.
        """
        c = self._count
        self._count = 0
        return c
