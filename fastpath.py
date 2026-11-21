"""Native-code (viper) versions of the firmware's hottest byte loops.

MicroPython only: CPython cannot import this module (there is no
``micropython``), and the callers then use their plain-Python versions,
which do exactly the same thing. On the ESP32 the callers compare both
versions once at start-up and fall back if they ever disagree.

  yd_scan / ld_scan   check one lidar packet's checksum and bin its points
                      (lidar_ld06.py); ~4000 points/s is too much for
                      interpreted Python
  mask                XOR a WebSocket frame with its mask (ws_client.py)
"""

import micropython


def _crc_table():
    """LD06 CRC8 table (poly 0x4D), as in lidar_ld06."""
    t = bytearray(256)
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0x4D) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        t[i] = c
    return t


_CRC = _crc_table()

# viper functions take at most 4 arguments


@micropython.viper
def yd_scan(buf: ptr8, i: int, ranges: ptr16, nbins: int) -> int:
    """YDLIDAR packet at ``buf[i]``: bin its points into ``ranges`` (mm, nearest per bin).

    Returns the start angle (1/64 deg), or -1 if the checksum is wrong.
    """
    lsn = buf[i + 3]
    cs = 0x55AA ^ (buf[i + 2] | (buf[i + 3] << 8))
    cs ^= buf[i + 4] | (buf[i + 5] << 8)
    cs ^= buf[i + 6] | (buf[i + 7] << 8)
    off = i + 10
    k = 0
    while k < lsn:
        cs ^= buf[off]
        cs ^= buf[off + 1] | (buf[off + 2] << 8)
        off += 3
        k += 1
    if cs != (buf[i + 8] | (buf[i + 9] << 8)):
        return -1
    full = 23040
    start = (buf[i + 4] | (buf[i + 5] << 8)) >> 1
    end = (buf[i + 6] | (buf[i + 7] << 8)) >> 1
    span = end - start
    while span < 0:
        span += full
    div = lsn - 1
    if div < 1:
        div = 1
    off = i + 10
    k = 0
    while k < lsn:
        d = (buf[off + 1] | (buf[off + 2] << 8)) >> 2
        if d > 0:
            a = start + span * k // div
            while a >= full:
                a -= full
            b = a * nbins // full
            r = ranges[b]
            if r == 0:
                ranges[b] = d
            elif d < r:
                ranges[b] = d
        off += 3
        k += 1
    return start


@micropython.viper
def ld_scan(buf: ptr8, i: int, ranges: ptr16, nbins: int) -> int:
    """LD06 packet at ``buf[i]``: like yd_scan; start angle in 0.01 deg, or -1 on a bad CRC."""
    table = ptr8(_CRC)
    crc = 0
    j = i
    e = i + 46
    while j < e:
        crc = table[(crc ^ buf[j]) & 0xFF]
        j += 1
    if crc != buf[e]:
        return -1
    full = 36000
    start = buf[i + 4] | (buf[i + 5] << 8)
    end = buf[i + 42] | (buf[i + 43] << 8)
    span = end - start
    while span < 0:
        span += full
    off = i + 6
    k = 0
    while k < 12:
        d = buf[off] | (buf[off + 1] << 8)
        if d > 0:
            a = start + span * k // 11
            while a >= full:
                a -= full
            b = a * nbins // full
            r = ranges[b]
            if r == 0:
                ranges[b] = d
            elif d < r:
                ranges[b] = d
        off += 3
        k += 1
    return start


@micropython.viper
def mask(buf: ptr8, start: int, n: int, m: ptr8):
    """XOR ``buf[start:n]`` in place with the repeating 4-byte mask ``m``."""
    i = start
    k = 0
    while i < n:
        buf[i] = buf[i] ^ m[k]
        k = (k + 1) & 3
        i += 1
