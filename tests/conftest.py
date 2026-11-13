"""Run the firmware modules on desktop Python.

MicroPython's ``machine`` module does not exist here, so a fake one records
what the code does to pins and PWM channels. The repository root goes on
``sys.path`` because the firmware imports its modules flat (``import config``).
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


class FakePin:
    OUT = 1
    IN = 0
    PULL_UP = 2
    IRQ_RISING = 1
    IRQ_FALLING = 2

    def __init__(self, num, mode=None, pull=None):
        self.num = num
        self._value = 0

    def value(self, v=None):
        if v is None:
            return self._value
        self._value = v

    def irq(self, *args, **kwargs):
        pass


class FakePWM:
    def __init__(self, pin, freq=None, duty=0):
        self.pin = pin
        self.freq = freq
        self._duty = duty

    def duty(self, d=None):
        if d is None:
            return self._duty
        self._duty = d


machine = types.ModuleType("machine")
machine.Pin = FakePin
machine.PWM = FakePWM
machine.reset = lambda: None
sys.modules.setdefault("machine", machine)


# MicroPython's StreamWriter has aclose(); CPython's has close() + wait_closed().
import asyncio  # noqa: E402

if not hasattr(asyncio.StreamWriter, "aclose"):
    async def _aclose(self):
        self.close()
        try:
            await self.wait_closed()
        except Exception:
            pass
    asyncio.StreamWriter.aclose = _aclose
