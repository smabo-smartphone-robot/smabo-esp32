"""Quadrature encoder counting using GPIO interrupts.

NOTE: This is a software (IRQ) decoder and *will* lose counts at high speed
or high resolution.  For reliable counting use a MicroPython build that
exposes the ESP32 PCNT peripheral and swap this class out.

Also note: ESP32 GPIO 34-39 are input-only with NO internal pull resistors,
so the default encoder pins need external pull-ups.
"""

from machine import Pin


class QuadEncoder:
    """Software x4 quadrature decoder driven by GPIO edge interrupts."""

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
        self._a = Pin(pin_a, Pin.IN)
        self._b = Pin(pin_b, Pin.IN)
        self._a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_a)
        self._b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._on_b)

    def _on_a(self, _pin):
        """Handle an edge on channel A, incrementing/decrementing the count.

        Parameters
        ----------
        _pin : machine.Pin
            The triggering pin (supplied by the IRQ, unused).

        Returns
        -------
        None
        """
        a = self._a.value()
        b = self._b.value()
        # standard x4-ish decode
        if a != b:
            self._count += 1
        else:
            self._count -= 1

    def _on_b(self, _pin):
        """Handle an edge on channel B, incrementing/decrementing the count.

        Parameters
        ----------
        _pin : machine.Pin
            The triggering pin (supplied by the IRQ, unused).

        Returns
        -------
        None
        """
        a = self._a.value()
        b = self._b.value()
        if a == b:
            self._count += 1
        else:
            self._count -= 1

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
