"""LD06 lidar driver → sensor_msgs/LaserScan publisher.

Reads the LD06's UART stream, parses its 47-byte packets (12 points each),
bins one full revolution into a fixed-size ``ranges`` array and publishes a
``/scan`` message (rosbridge JSON). smabo-brain-ros / Nav2 consume it as the
costmap / SLAM sensor source.

The LD06 connects to the MCU directly over UART (default 230400 8N1); only the
lidar's TX → MCU RX line is needed. Pose/odometry stay on the encoder path; this
module is independent and gated by ``modes.lidar``.

Packet layout (little-endian):
  0      header   0x54
  1      verlen   0x2C  (low 5 bits = 12 points)
  2-3    speed    deg/s
  4-5    start    angle * 0.01 deg
  6-41   12 x (distance uint16 mm, intensity uint8)
  42-43  end      angle * 0.01 deg
  44-45  timestamp ms
  46     crc8
"""

import math

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

try:
    import time
    _ticks_ms = time.ticks_ms
except AttributeError:
    import time
    _ticks_ms = lambda: int(time.time() * 1000)

_HEADER = 0x54
_VERLEN = 0x2C
_POINTS = 12
_PKT_LEN = 47


def _gen_crc_table():
    """LD06 CRC8 table (poly 0x4D, MSB-first, init 0)."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        table.append(c)
    return table


_CRC_TABLE = _gen_crc_table()


def _crc8(data):
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


class Ld06:
    """Parse the LD06 UART stream and publish /scan once per revolution."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._buf = bytearray()
        self._nbins = int(cfg.get("lidar.bins", 360)) or 360
        self._ranges = [0.0] * self._nbins
        self._prev_start = None
        self._last_emit_ms = _ticks_ms()
        self.uart = self._open_uart()

    def _open_uart(self):
        """Open the LD06 UART, or return None off-target / on error."""
        try:
            from machine import UART, Pin
        except ImportError:
            print("LD06: machine.UART unavailable (off-target); idling")
            return None
        try:
            uart_id = self.cfg.get("lidar.uart", 1)
            rx = self.cfg.get("lidar.rx", 20)
            tx = self.cfg.get("lidar.tx", -1)
            baud = self.cfg.get("lidar.baud", 230400)
            kwargs = dict(baudrate=baud, bits=8, parity=None, stop=1,
                          rx=Pin(rx), rxbuf=1024, timeout=0)
            if tx is not None and tx >= 0:
                kwargs["tx"] = Pin(tx)
            return UART(uart_id, **kwargs)
        except Exception as e:
            print("LD06: UART init failed:", e)
            return None

    def deinit(self):
        """Release the UART (called on subsystem teardown / pin change)."""
        try:
            if self.uart is not None:
                self.uart.deinit()
        except Exception:
            pass
        self.uart = None

    # ------------------------------------------------------------------ #
    async def run(self, publish):
        """Read, parse and publish ``/scan`` forever (idle if no UART)."""
        while True:
            if self.uart is None:
                await asyncio.sleep(1.0)
                continue
            n = self.uart.any()
            if n:
                chunk = self.uart.read(n)
                if chunk:
                    self._buf.extend(chunk)
                    self._consume(publish)
            await asyncio.sleep_ms(5)

    # ------------------------------------------------------------------ #
    def _consume(self, publish):
        """Extract complete CRC-valid packets from the byte buffer."""
        buf = self._buf
        i = 0
        n = len(buf)
        while n - i >= _PKT_LEN:
            if buf[i] != _HEADER or buf[i + 1] != _VERLEN:
                i += 1
                continue
            pkt = buf[i:i + _PKT_LEN]
            if _crc8(pkt[:_PKT_LEN - 1]) != pkt[_PKT_LEN - 1]:
                i += 1            # bad CRC → resync one byte forward
                continue
            self._parse_packet(pkt, publish)
            i += _PKT_LEN
        # keep the unconsumed tail
        if i:
            self._buf = buf[i:]

    def _parse_packet(self, pkt, publish):
        start = (pkt[4] | (pkt[5] << 8)) / 100.0          # deg
        end = (pkt[42] | (pkt[43] << 8)) / 100.0          # deg
        span = (end - start) % 360.0
        step = span / (_POINTS - 1) if _POINTS > 1 else 0.0

        for k in range(_POINTS):
            off = 6 + k * 3
            dist_mm = pkt[off] | (pkt[off + 1] << 8)
            ang = (start + step * k) % 360.0
            b = int(ang * self._nbins / 360.0) % self._nbins
            if dist_mm > 0:
                d = dist_mm / 1000.0
                # keep the nearest return in each bin
                if self._ranges[b] == 0.0 or d < self._ranges[b]:
                    self._ranges[b] = d

        # a revolution completes when the start angle wraps past 0
        if self._prev_start is not None and start < self._prev_start:
            self._emit(publish)
        self._prev_start = start

    def _emit(self, publish):
        now_ms = _ticks_ms()
        scan_time = (now_ms - self._last_emit_ms) / 1000.0
        self._last_emit_ms = now_ms
        inc = 2.0 * math.pi / self._nbins
        stamp = {"sec": now_ms // 1000, "nanosec": (now_ms % 1000) * 1_000_000}

        publish("/scan", {
            "header": {"stamp": stamp, "frame_id": self.cfg.get("lidar.frame_id", "laser")},
            "angle_min": 0.0,
            "angle_max": inc * (self._nbins - 1),
            "angle_increment": inc,
            "time_increment": (scan_time / self._nbins) if scan_time > 0 else 0.0,
            "scan_time": scan_time if scan_time > 0 else 0.1,
            "range_min": self.cfg.get("lidar.range_min", 0.05),
            "range_max": self.cfg.get("lidar.range_max", 12.0),
            "ranges": list(self._ranges),       # 0.0 = no return (ignored by Nav2)
            "intensities": [],
        })
        # reset for the next revolution
        for j in range(self._nbins):
            self._ranges[j] = 0.0
