"""Minimal PCA9685 16-channel PWM driver (I2C).

Only what we need for servos: set the frequency once, then push a pulse
width in microseconds per channel.
"""

_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06


class PCA9685:
    """PCA9685 16-channel 12-bit PWM controller on an I2C bus."""

    def __init__(self, i2c, address=0x40, freq=50):
        """Bind to an I2C bus, reset the chip and set the PWM frequency.

        Parameters
        ----------
        i2c : machine.I2C
            An already-initialised I2C bus instance.
        address : int, optional
            7-bit I2C address of the PCA9685 (default ``0x40``).
        freq : int, optional
            PWM frequency in Hz applied to all 16 channels (default ``50``).
        """
        self.i2c = i2c
        self.addr = address
        self._buf1 = bytearray(1)
        self._buf2 = bytearray(2)
        self.reset()
        self._freq = freq
        self.set_freq(freq)

    def _write8(self, reg, value):
        """Write one byte to a device register.

        Parameters
        ----------
        reg : int
            Register address.
        value : int
            Byte to write (only the low 8 bits are used).

        Returns
        -------
        None
        """
        self._buf2[0] = reg
        self._buf2[1] = value & 0xFF
        self.i2c.writeto(self.addr, self._buf2)

    def _read8(self, reg):
        """Read one byte from a device register.

        Parameters
        ----------
        reg : int
            Register address.

        Returns
        -------
        int
            The byte read from ``reg`` (0-255).
        """
        self._buf1[0] = reg
        self.i2c.writeto(self.addr, self._buf1)
        return self.i2c.readfrom(self.addr, 1)[0]

    def reset(self):
        """Reset the MODE1 register to its power-on default.

        Returns
        -------
        None
        """
        self._write8(_MODE1, 0x00)

    def set_freq(self, freq):
        """Set the global PWM frequency via the prescaler.

        Parameters
        ----------
        freq : int
            Target PWM frequency in Hz (applies to all 16 channels).

        Returns
        -------
        None
        """
        self._freq = freq
        prescale = int(round(25000000.0 / (4096 * freq))) - 1
        if prescale < 3:
            prescale = 3
        old = self._read8(_MODE1)
        self._write8(_MODE1, (old & 0x7F) | 0x10)  # sleep
        self._write8(_PRESCALE, prescale)
        self._write8(_MODE1, old)
        # wait >500us for oscillator, then enable auto-increment + restart
        for _ in range(1000):
            pass
        self._write8(_MODE1, old | 0xA1)

    def set_pwm(self, channel, on, off):
        """Set the raw on/off tick counts for one channel.

        Parameters
        ----------
        channel : int
            Output channel index (0-15).
        on : int
            Tick within the 4096-step period at which the output goes high
            (0-4095).
        off : int
            Tick at which the output goes low (0-4095).

        Returns
        -------
        None
        """
        reg = _LED0_ON_L + 4 * channel
        data = bytearray(5)
        data[0] = reg
        data[1] = on & 0xFF
        data[2] = (on >> 8) & 0xFF
        data[3] = off & 0xFF
        data[4] = (off >> 8) & 0xFF
        self.i2c.writeto(self.addr, data)

    def set_us(self, channel, us):
        """Set the high-pulse width of a channel in microseconds.

        The width is converted to tick counts using the current frequency and
        clamped to the valid 0-4095 range.

        Parameters
        ----------
        channel : int
            Output channel index (0-15).
        us : float
            Desired high-pulse width in microseconds.

        Returns
        -------
        None
        """
        period_us = 1000000.0 / self._freq
        ticks = int(round(us / period_us * 4096))
        if ticks < 0:
            ticks = 0
        elif ticks > 4095:
            ticks = 4095
        self.set_pwm(channel, 0, ticks)

    def off(self, channel):
        """Cut PWM output completely (FULL_OFF bit) so the servo goes free.

        Parameters
        ----------
        channel : int
            Output channel index (0-15).

        Returns
        -------
        None
        """
        self.set_pwm(channel, 0, 0x1000)  # bit 12 = LED_FULL_OFF
