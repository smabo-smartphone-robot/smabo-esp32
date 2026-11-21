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

Some units sold as LD19 (clones) speak the YDLIDAR protocol instead. The
driver recognises either format from the stream (each packet is checked by
its own CRC / checksum) and prints which one it found:
  0-1    header   0xAA 0x55
  2      CT       bit0 = 1 on the first packet of a revolution
  3      LSN      number of samples
  4-5    FSA      first angle: (raw >> 1) / 64 deg
  6-7    LSA      last angle:  (raw >> 1) / 64 deg
  8-9    CS       XOR of the 16-bit words of the packet
  10-    LSN x (intensity uint8, distance uint16: mm = raw >> 2)

The ESP32 has little RAM, so the hot path allocates nothing: bytes are read
into one fixed buffer and parsed in place with integer maths, ranges are kept
in mm in an ``array('H')``, and each /scan is written as JSON text into a
reused buffer that is handed to ``publish`` as bytes (brain_link sends it
as the message body without re-encoding it).
"""

import gc
from array import array

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

try:
    _ticks_diff = time.ticks_diff
except AttributeError:
    _ticks_diff = lambda a, b: a - b

_HEADER = 0x54
_VERLEN = 0x2C
_POINTS = 12
_PKT_LEN = 47
_LD_FULL = 36000          # LD06 angles: 0.01 deg units

# YDLIDAR-format packets (some LD19 clones)
_YD_HEAD0 = 0xAA
_YD_HEAD1 = 0x55
_YD_HDR_LEN = 10
_YD_MAX_LSN = 80          # anything longer is a false header
_YD_FULL = 360 * 64       # YDLIDAR angles: 1/64 deg units

# UART driver buffer (outside the Python heap): ~0.15 s of data at 230400 baud
_RXBUF = 2048
# our parse buffer; must hold the longest packet (10 + 3 * 80 bytes)
_PARSE_BUF = 1024

# Minimum time between /scan messages. Every revolution should be its own scan:
# the brain stamps a scan when it arrives, so merged revolutions would hand
# SLAM points up to several revolutions old (smeared while turning). 90 ms
# lets lidars up to ~11 Hz (LD19 ~10 Hz, YDLIDAR-type ~6 Hz) through one by
# one, and only merges revolutions if something spins (or glitches) faster.
_MIN_EMIT_MS = 90


def _gen_crc_table():
    """LD06 CRC8 table (poly 0x4D, MSB-first, init 0)."""
    table = bytearray(256)
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        table[i] = c
    return table


_CRC_TABLE = _gen_crc_table()


def _crc8(data, start=0, end=None):
    """CRC8 of ``data[start:end]``."""
    if end is None:
        end = len(data)
    crc = 0
    table = _CRC_TABLE
    for i in range(start, end):
        crc = table[(crc ^ data[i]) & 0xFF]
    return crc


def _yd_checksum(pkt, lsn, i=0):
    """YDLIDAR checksum of the packet at ``pkt[i:]``: XOR of its header words and samples."""
    cs = 0x55AA ^ (pkt[i + 2] | (pkt[i + 3] << 8))
    cs ^= pkt[i + 4] | (pkt[i + 5] << 8)
    cs ^= pkt[i + 6] | (pkt[i + 7] << 8)
    off = i + _YD_HDR_LEN
    for _ in range(lsn):
        cs ^= pkt[off]                              # intensity
        cs ^= pkt[off + 1] | (pkt[off + 2] << 8)    # distance
        off += 3
    return cs


def _py_yd_scan(buf, i, ranges, nbins):
    """YDLIDAR packet at ``buf[i]``: bin its points into ``ranges`` (mm, nearest per bin).

    Returns the start angle (1/64 deg), or -1 if the checksum is wrong.
    Same as fastpath.yd_scan, in plain Python.
    """
    lsn = buf[i + 3]
    if _yd_checksum(buf, lsn, i) != (buf[i + 8] | (buf[i + 9] << 8)):
        return -1
    full = _YD_FULL
    start = (buf[i + 4] | (buf[i + 5] << 8)) >> 1
    end = (buf[i + 6] | (buf[i + 7] << 8)) >> 1
    span = end - start
    while span < 0:
        span += full
    div = lsn - 1 if lsn > 1 else 1
    off = i + _YD_HDR_LEN
    for k in range(lsn):
        d = (buf[off + 1] | (buf[off + 2] << 8)) >> 2
        if d > 0:
            a = start + span * k // div
            while a >= full:
                a -= full
            b = a * nbins // full
            r = ranges[b]
            if r == 0 or d < r:
                ranges[b] = d
        off += 3
    return start


def _py_ld_scan(buf, i, ranges, nbins):
    """LD06 packet at ``buf[i]``: like _py_yd_scan; start angle in 0.01 deg, or -1 on a bad CRC."""
    if _crc8(buf, i, i + _PKT_LEN - 1) != buf[i + _PKT_LEN - 1]:
        return -1
    full = _LD_FULL
    start = buf[i + 4] | (buf[i + 5] << 8)
    end = buf[i + 42] | (buf[i + 43] << 8)
    span = end - start
    while span < 0:
        span += full
    off = i + 6
    for k in range(_POINTS):
        d = buf[off] | (buf[off + 1] << 8)
        if d > 0:
            a = start + span * k // (_POINTS - 1)
            while a >= full:
                a -= full
            b = a * nbins // full
            r = ranges[b]
            if r == 0 or d < r:
                ranges[b] = d
        off += 3
    return start


def _check_packets():
    """An LD06 and a YDLIDAR packet (crossing 0 deg, with gaps) for the self-check."""
    ld = bytearray(_PKT_LEN)
    ld[0], ld[1] = _HEADER, _VERLEN
    ld[4], ld[5] = 35500 & 0xFF, 35500 >> 8
    ld[42], ld[43] = 600 & 0xFF, 600 >> 8
    for k in range(_POINTS):
        d = 0 if k == 3 else 300 + 37 * k
        ld[6 + 3 * k], ld[7 + 3 * k] = d & 0xFF, d >> 8
    ld[_PKT_LEN - 1] = _crc8(ld, 0, _PKT_LEN - 1)
    lsn = 25
    yd = bytearray(_YD_HDR_LEN + 3 * lsn)
    yd[0], yd[1], yd[3] = _YD_HEAD0, _YD_HEAD1, lsn
    fsa, lsa = ((350 * 64) << 1) | 1, ((14 * 64) << 1) | 1
    yd[4], yd[5], yd[6], yd[7] = fsa & 0xFF, fsa >> 8, lsa & 0xFF, lsa >> 8
    for k in range(lsn):
        raw = (0 if k == 5 else 500 + 101 * k) << 2
        o = _YD_HDR_LEN + 3 * k
        yd[o], yd[o + 1], yd[o + 2] = 90, raw & 0xFF, raw >> 8
    cs = _yd_checksum(yd, lsn)
    yd[8], yd[9] = cs & 0xFF, cs >> 8
    return ld, yd, lsn


def _pick_scanners():
    """The viper scanners if they load and agree with the Python ones, else the Python ones."""
    try:
        import fastpath
        ld, yd, lsn = _check_packets()
        for nbins in (360, 720):
            a = array("H", bytes(2 * nbins))
            b = array("H", bytes(2 * nbins))
            if (fastpath.ld_scan(ld, 0, a, nbins) != _py_ld_scan(ld, 0, b, nbins)
                    or fastpath.yd_scan(yd, 0, a, nbins) != _py_yd_scan(yd, 0, b, nbins)
                    or bytes(a) != bytes(b)):
                print("LD06: native scanner disagrees; using Python")
                return _py_yd_scan, _py_ld_scan
        yd[20] ^= 1                           # a bad checksum must be refused too
        if fastpath.yd_scan(yd, 0, a, 360) != -1:
            print("LD06: native scanner disagrees; using Python")
            return _py_yd_scan, _py_ld_scan
        return fastpath.yd_scan, fastpath.ld_scan
    except Exception:
        return _py_yd_scan, _py_ld_scan


# Chosen on first use (Ld06()), not at import: robot.py imports this module
# before Wi-Fi starts, and native code takes internal RAM that the Wi-Fi
# driver needs for its buffers at start-up.
_yd_scan = _ld_scan = None


def _load_scanners():
    global _yd_scan, _ld_scan
    if _yd_scan is None:
        _yd_scan, _ld_scan = _pick_scanners()


def _fmt(x):
    """A float as short JSON text (always with a decimal point)."""
    t = "%.6g" % x
    return t if ("." in t or "e" in t) else t + ".0"


class Ld06:
    """Parse the lidar's UART stream and publish /scan once per revolution."""

    def __init__(self, cfg):
        self.cfg = cfg
        _load_scanners()
        self._nbins = int(cfg.get("lidar.bins")) or 360
        self._ranges = array("H", bytes(2 * self._nbins))     # mm, 0 = no return
        self._rx = bytearray(_PARSE_BUF)
        self._rx_mv = memoryview(self._rx)
        self._n = 0                      # bytes waiting in _rx
        # JSON text of one /scan: fixed part + up to 7 chars per range
        self._out = bytearray(512 + 7 * self._nbins)
        self._prev_start = None          # start angle of the last packet, in 1/_full deg
        self._full = _LD_FULL
        self.protocol = None             # "ld06" or "ydlidar", once a valid packet is seen
        self._last_emit_ms = _ticks_ms()
        self.uart = self._open_uart()

    def _open_uart(self):
        """Open the lidar UART, or return None off-target / on error."""
        try:
            from machine import UART, Pin
        except ImportError:
            print("LD06: machine.UART unavailable (off-target); idling")
            return None
        try:
            uart_id = self.cfg.get("lidar.uart")
            rx = self.cfg.get("lidar.rx")
            tx = self.cfg.get("lidar.tx")
            baud = self.cfg.get("lidar.baud")
            kwargs = dict(baudrate=baud, bits=8, parity=None, stop=1,
                          rx=Pin(rx), rxbuf=_RXBUF, timeout=0)
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
            if self.uart.any():
                try:
                    got = self.uart.readinto(self._rx_mv[self._n:])
                    if got:
                        self._n += got
                        self._consume(publish)
                except MemoryError:
                    # drop this revolution and carry on rather than let
                    # the lidar task die
                    self._n = 0
                    self._reset_ranges()
                    gc.collect()
                    print("LD06: out of memory, scan dropped")
            await asyncio.sleep_ms(5)

    def feed(self, data, publish):
        """Push bytes through the parser (what ``run`` does with UART reads)."""
        k = 0
        while k < len(data):
            room = _PARSE_BUF - self._n
            take = min(room, len(data) - k)
            self._rx[self._n:self._n + take] = data[k:k + take]
            self._n += take
            k += take
            self._consume(publish)

    # ------------------------------------------------------------------ #
    def _consume(self, publish):
        """Parse every complete, checksum-valid packet (LD06 or YDLIDAR format)."""
        buf = self._rx
        n = self._n
        i = 0
        while n - i >= 2:
            b0 = buf[i]
            b1 = buf[i + 1]
            if b0 == _HEADER and b1 == _VERLEN:
                if n - i < _PKT_LEN:
                    break                     # wait for the rest
                start = _ld_scan(buf, i, self._ranges, self._nbins)
                if start < 0:
                    i += 1                    # bad CRC → resync one byte forward
                    continue
                self._found("ld06", _LD_FULL)
                self._end_packet(start, publish)
                i += _PKT_LEN
            elif b0 == _YD_HEAD0 and b1 == _YD_HEAD1:
                if n - i < _YD_HDR_LEN:
                    break
                lsn = buf[i + 3]
                if lsn == 0 or lsn > _YD_MAX_LSN:
                    i += 1
                    continue
                size = _YD_HDR_LEN + 3 * lsn
                if n - i < size:
                    break
                start = _yd_scan(buf, i, self._ranges, self._nbins)
                if start < 0:
                    i += 1
                    continue
                self._found("ydlidar", _YD_FULL)
                self._end_packet(start, publish)
                i += size
            else:
                i += 1
        # move the unconsumed tail to the front (it is shorter than one packet)
        if i:
            rest = n - i
            for k in range(rest):
                buf[k] = buf[i + k]
            self._n = rest
        elif n >= _PARSE_BUF:
            self._n = 0                       # full of junk: start over

    def _found(self, protocol, full):
        if self.protocol != protocol:
            print("LD06: detected", protocol, "packets",
                  "(native)" if _yd_scan is not _py_yd_scan else "(python)")
            self.protocol = protocol
            self._full = full
            self._prev_start = None

    def _end_packet(self, start, publish):
        # a revolution completes when the start angle wraps past 0 (e.g.
        # 355 -> 3 deg); a small step back (overlapping packets, a clone's
        # start-of-revolution packet) is not a wrap and would publish a
        # near-empty scan
        if self._prev_start is not None and self._prev_start - start > self._full // 2:
            if _ticks_diff(_ticks_ms(), self._last_emit_ms) >= _MIN_EMIT_MS:
                self._emit(publish)
        self._prev_start = start

    def _emit(self, publish):
        now_ms = _ticks_ms()
        scan_time = _ticks_diff(now_ms, self._last_emit_ms) / 1000.0
        self._last_emit_ms = now_ms
        nb = self._nbins
        inc = 6.283185307179586 / nb
        if scan_time <= 0:
            scan_time = 0.1
        # The lidar's raw angles (bin k = k * 360/nb deg) are turned into the
        # robot's convention: ROS angle = dir * raw + angle_offset, with
        # dir = -1 for lidars that measure clockwise (LD06/LD19, YDLIDAR;
        # LaserScan is counter-clockwise). Whole bins of the offset shift the
        # array; the remainder goes into angle_min, so the offset is exact.
        step = -1 if self.cfg.get("lidar.clockwise") else 1
        bin_deg = 360.0 / nb
        offset = float(self.cfg.get("lidar.angle_offset") or 0.0)
        shift = int(offset / bin_deg)
        if shift * bin_deg > offset:
            shift -= 1                       # floor, also for negative offsets
        angle_min = (offset - shift * bin_deg) * 0.017453292519943295
        head = (
            '{"header":{"stamp":{"sec":%d,"nanosec":%d},"frame_id":"%s"},'
            '"angle_min":%s,"angle_max":%s,"angle_increment":%s,'
            '"time_increment":%s,"scan_time":%s,"range_min":%s,"range_max":%s,'
            '"ranges":[' % (
                now_ms // 1000, (now_ms % 1000) * 1000000,
                self.cfg.get("lidar.frame_id"),
                _fmt(angle_min), _fmt(angle_min + inc * (nb - 1)), _fmt(inc),
                _fmt(scan_time / nb), _fmt(scan_time),
                _fmt(self.cfg.get("lidar.range_min")),
                _fmt(self.cfg.get("lidar.range_max")))
        ).encode()
        out = self._out
        p = len(head)
        out[:p] = head
        r = self._ranges
        k = (-shift * step) % nb            # raw bin shown at output index 0
        for j in range(nb):
            if j:
                out[p] = 44                 # ,
                p += 1
            p = _put_m(out, p, r[k])        # 0.0 = no return (ignored by Nav2)
            k += step
            if k == nb:
                k = 0
            elif k < 0:
                k = nb - 1
        tail = b'],"intensities":[]}'
        out[p:p + len(tail)] = tail
        p += len(tail)
        publish("/scan", memoryview(out)[:p])
        self._reset_ranges()

    def _reset_ranges(self):
        """Start the next revolution with no returns."""
        r = self._ranges
        for j in range(self._nbins):
            r[j] = 0


def _put_m(out, p, mm):
    """Write ``mm`` as metres ("1.234", "0.0") into ``out`` at ``p``; the new position."""
    if mm == 0:
        out[p] = 48                         # 0
        out[p + 1] = 46                     # .
        out[p + 2] = 48
        return p + 3
    m = mm // 1000
    f = mm % 1000
    if m >= 10:
        out[p] = 48 + m // 10
        p += 1
    out[p] = 48 + m % 10
    out[p + 1] = 46
    out[p + 2] = 48 + f // 100
    out[p + 3] = 48 + (f // 10) % 10
    out[p + 4] = 48 + f % 10
    return p + 5
