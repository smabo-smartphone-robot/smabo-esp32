"""lidar_ld06: LD06/LD19 and YDLIDAR-format packets are told apart and parsed."""

import json
import math

from conftest import defaults_at
from lidar_ld06 import Ld06, _crc8, _yd_checksum

# One YDLIDAR-format packet captured from a unit sold as LD19 (85 bytes).
YD_CAPTURED = bytes.fromhex(
    "aa55 0019 dd22 e32d 8a4a"
    "d08308 cc8308 cc8f08 e4e308 f0e308 e4df08 e0d708 e0cf08 e4d708 d4df08"
    "d0ff08 cc2309 c84f09 c88309 bcb309 ccef09 c83f0a 50d709 647f09 38bf09"
    "20ed09 bcc90f b08d0e 94310e 90320d"
)


class Cfg(dict):
    def get(self, key, default=None):
        return super().get(key, defaults_at(key, default))


def make_lidar(**cfg):
    # raw angles unless a test asks otherwise, so bins read straight off the packets
    conf = {"lidar.bins": 360, "lidar.clockwise": False}
    conf.update({"lidar." + k: v for k, v in cfg.items()})
    lidar = Ld06(Cfg(conf))
    assert lidar.uart is None            # off-target: no UART
    lidar._last_emit_ms -= 1000          # as if the last scan went out a while ago
    return lidar


def ld06_packet(start_deg, end_deg, dist_mm):
    pkt = bytearray([0x54, 0x2C, 0, 0])
    s = int(start_deg * 100)
    pkt += bytes([s & 0xFF, s >> 8])
    for _ in range(12):
        pkt += bytes([dist_mm & 0xFF, dist_mm >> 8, 200])
    e = int(end_deg * 100)
    pkt += bytes([e & 0xFF, e >> 8, 0, 0])
    pkt.append(_crc8(pkt))
    return bytes(pkt)


def yd_packet(start_deg, end_deg, dists_mm, ct=0):
    lsn = len(dists_mm)
    fsa = (int(start_deg * 64) << 1) | 1
    lsa = (int(end_deg * 64) << 1) | 1
    pkt = bytearray([0xAA, 0x55, ct, lsn, fsa & 0xFF, fsa >> 8, lsa & 0xFF, lsa >> 8, 0, 0])
    for d in dists_mm:
        raw = d << 2
        pkt += bytes([100, raw & 0xFF, raw >> 8])
    cs = _yd_checksum(pkt, lsn)
    pkt[8], pkt[9] = cs & 0xFF, cs >> 8
    return bytes(pkt)


def feed(lidar, data):
    out = []
    lidar.feed(data, lambda topic, msg: out.append((topic, json.loads(bytes(msg)))))
    return out


def test_captured_ydlidar_packet_is_recognised_and_binned():
    lidar = make_lidar()
    assert feed(lidar, b"\x00\x13garbage" + YD_CAPTURED) == []
    assert lidar.protocol == "ydlidar"
    assert lidar._n == 0
    # first sample: 69.7 deg, 544 mm; last: 91.8 deg, 844 mm
    assert lidar._ranges[69] == 544
    assert lidar._ranges[91] == 844
    assert sum(1 for r in lidar._ranges if r) >= 20
    assert all(r == 0 for b, r in enumerate(lidar._ranges) if not 69 <= b <= 91)


def test_corrupt_ydlidar_packet_is_dropped():
    lidar = make_lidar()
    bad = bytearray(YD_CAPTURED)
    bad[20] ^= 0xFF
    feed(lidar, bytes(bad))
    assert lidar.protocol is None
    assert not any(lidar._ranges)


def test_partial_packet_waits_for_the_rest():
    lidar = make_lidar()
    feed(lidar, YD_CAPTURED[:40])
    assert lidar.protocol is None and lidar._n == 40
    feed(lidar, YD_CAPTURED[40:])
    assert lidar.protocol == "ydlidar"


def test_ydlidar_revolution_publishes_scan():
    lidar = make_lidar()
    out = feed(lidar, yd_packet(10, 20, [1000] * 11)
               + yd_packet(350, 359, [2000] * 10)
               + yd_packet(1, 9, [500] * 9, ct=1))
    assert [t for t, _ in out] == ["/scan"]
    ranges = out[0][1]["ranges"]
    assert ranges[10] == 1.0 and ranges[20] == 1.0 and ranges[355] == 2.0
    assert abs(out[0][1]["angle_increment"] - 2 * math.pi / 360) < 1e-6


def test_ld06_packets_still_work():
    lidar = make_lidar()
    out = feed(lidar, ld06_packet(100, 110, 1500)
               + ld06_packet(350, 359, 800)
               + ld06_packet(2, 12, 900))
    assert lidar.protocol == "ld06"
    assert [t for t, _ in out] == ["/scan"]
    ranges = out[0][1]["ranges"]
    assert ranges[100] == 1.5 and ranges[355] == 0.8


def test_small_step_back_is_not_a_new_revolution():
    lidar = make_lidar()
    out = feed(lidar, yd_packet(100, 110, [1000] * 11)
               + yd_packet(108, 118, [1000] * 11)      # overlaps the previous one
               + yd_packet(0, 0, [1200], ct=1)          # off-cycle start packet
               + yd_packet(120, 130, [1000] * 11))
    assert out == []
    out = feed(lidar, yd_packet(350, 359, [900] * 10) + yd_packet(2, 12, [900] * 11))
    assert [t for t, _ in out] == ["/scan"]


def test_scans_are_rate_limited_and_merged():
    lidar = make_lidar()
    rev = yd_packet(10, 20, [1000] * 11) + yd_packet(350, 359, [2000] * 10)
    out = feed(lidar, rev + rev)              # two wraps back to back
    assert len(out) == 1                      # the second came too soon
    out = feed(lidar, yd_packet(100, 110, [700] * 11) + yd_packet(2, 12, [900] * 11))
    assert out == []
    lidar._last_emit_ms -= 1000
    out = feed(lidar, yd_packet(350, 359, [2000] * 10) + yd_packet(2, 12, [900] * 11))
    ranges = out[0][1]["ranges"]
    assert ranges[100] == 0.7 and ranges[355] == 2.0   # both revolutions are in it


def test_out_of_memory_drops_the_scan_but_keeps_running(monkeypatch):
    import asyncio

    # MicroPython's asyncio has sleep_ms; CPython's does not
    monkeypatch.setattr(asyncio, "sleep_ms", lambda ms: asyncio.sleep(ms / 1000), raising=False)
    lidar = make_lidar()

    class Uart:
        def __init__(self, chunks):
            self.chunks = list(chunks)

        def any(self):
            return len(self.chunks[0]) if self.chunks else 0

        def readinto(self, buf):
            data = self.chunks.pop(0)
            buf[:len(data)] = data
            return len(data)

    def publish(topic, msg):
        raise MemoryError("memory allocation failed")

    lidar.uart = Uart([yd_packet(350, 359, [900] * 10) + yd_packet(2, 12, [900] * 11),
                       yd_packet(20, 30, [800] * 11)])

    async def go():
        task = asyncio.create_task(lidar.run(publish))
        await asyncio.sleep(0.05)
        task.cancel()
    asyncio.run(go())
    assert not lidar.uart.chunks                  # kept reading after the error
    assert lidar._ranges[20] == 800


def test_scan_json_is_a_valid_laserscan():
    lidar = make_lidar()
    out = feed(lidar, yd_packet(10, 20, [16383] * 11) + yd_packet(350, 359, [7] * 10)
               + yd_packet(2, 12, [900] * 11))
    msg = out[0][1]
    assert msg["header"]["frame_id"] == "laser"
    assert len(msg["ranges"]) == 360 and all(isinstance(r, float) for r in msg["ranges"])
    assert msg["ranges"][15] == 16.383 and msg["ranges"][355] == 0.007 and msg["ranges"][0] == 0.0
    assert abs(msg["angle_increment"] - 2 * math.pi / 360) < 1e-6
    assert msg["range_max"] == 12.0 and msg["intensities"] == []
    assert len(lidar._out) >= 512 + 7 * 360


def test_ld06_angles_interpolate_across_zero():
    lidar = make_lidar()
    feed(lidar, ld06_packet(355, 6, 1000))
    hit = [b for b, r in enumerate(lidar._ranges) if r]
    assert hit[0] == 0 and hit[-1] == 359 and 6 in hit


def test_native_scanners_are_used_only_if_they_agree(monkeypatch):
    import sys
    import types

    import lidar_ld06 as L

    # nothing is loaded at import (native code would take RAM before Wi-Fi starts)
    monkeypatch.setattr(L, "_yd_scan", None)
    monkeypatch.setattr(L, "_ld_scan", None)
    make_lidar()
    # CPython cannot load fastpath (no micropython): the Python scanners are used
    assert L._yd_scan is L._py_yd_scan and L._ld_scan is L._py_ld_scan

    good = types.SimpleNamespace(yd_scan=L._py_yd_scan, ld_scan=L._py_ld_scan)
    monkeypatch.setitem(sys.modules, "fastpath", good)
    assert L._pick_scanners() == (good.yd_scan, good.ld_scan)

    def off_by_one(buf, i, ranges, nbins):
        start = L._py_yd_scan(buf, i, ranges, nbins)
        if start >= 0:
            ranges[0] += 1
        return start

    bad = types.SimpleNamespace(yd_scan=off_by_one, ld_scan=L._py_ld_scan)
    monkeypatch.setitem(sys.modules, "fastpath", bad)
    assert L._pick_scanners() == (L._py_yd_scan, L._py_ld_scan)


def test_self_check_packets_cross_zero():
    import lidar_ld06 as L
    from array import array

    ld, yd, lsn = L._check_packets()
    r = array("H", bytes(720))
    assert L._py_ld_scan(ld, 0, r, 360) == 35500
    assert L._py_yd_scan(yd, 0, r, 360) == 350 * 64
    hit = [b for b, v in enumerate(r) if v]
    assert 0 in hit and 359 in hit and 5 in hit and 13 in hit


def test_each_revolution_is_its_own_scan_at_10hz(monkeypatch):
    import lidar_ld06 as L

    now = [10_000]
    monkeypatch.setattr(L, "_ticks_ms", lambda: now[0])
    lidar = make_lidar()
    lidar._last_emit_ms = now[0] - 1000
    scans = []
    for rev in range(4):                      # a 10 Hz lidar: one wrap every 100 ms
        now[0] += 100
        scans += feed(lidar, yd_packet(350, 359, [1000 + rev] * 10) + yd_packet(2, 12, [900] * 11))
    assert len(scans) == 4
    # each scan holds only its own revolution
    assert [s[1]["ranges"][355] for s in scans[1:]] == [1.001, 1.002, 1.003]


def one_point_scan(deg, **cfg):
    """Publish a scan whose only return is 1.5 m at raw angle ``deg``."""
    lidar = make_lidar(**cfg)
    feed(lidar, yd_packet(deg, deg, [1500]))
    lidar._last_emit_ms -= 1000
    out = feed(lidar, yd_packet(350, 350, [0]) + yd_packet(1, 1, [0]))
    msg = out[-1][1]
    hits = [i for i, r in enumerate(msg["ranges"]) if r]
    assert len(hits) == 1
    return math.degrees(msg["angle_min"] + hits[0] * msg["angle_increment"]) % 360


def test_clockwise_lidar_is_turned_counter_clockwise_by_default():
    lidar = Ld06(Cfg({"lidar.bins": 360}))
    feed(lidar, yd_packet(30, 30, [1500]))
    lidar._last_emit_ms -= 1000
    msg = feed(lidar, yd_packet(350, 350, [0]) + yd_packet(1, 1, [0]))[-1][1]
    hit = [i for i, r in enumerate(msg["ranges"]) if r]
    assert hit == [330]                               # 30 deg clockwise = -30 deg
    assert msg["angle_min"] == 0.0


def test_angle_offset_rotates_the_scan():
    assert abs(one_point_scan(30) - 30) < 1e-3
    assert abs(one_point_scan(30, clockwise=True) - 330) < 1e-3
    assert abs(one_point_scan(30, angle_offset=90) - 120) < 1e-3
    assert abs(one_point_scan(30, clockwise=True, angle_offset=-90) - 240) < 1e-3
    # a fraction of a bin goes into angle_min, exactly
    assert abs(one_point_scan(30, angle_offset=12.25) - 42.25) < 1e-3
    assert abs(one_point_scan(30, clockwise=True, angle_offset=-0.4) - 329.6) < 1e-3


def test_angle_min_and_max_span_one_turn():
    lidar = make_lidar(angle_offset=-0.4)
    feed(lidar, yd_packet(10, 20, [1000] * 11))
    lidar._last_emit_ms -= 1000
    msg = feed(lidar, yd_packet(350, 359, [0] * 10) + yd_packet(1, 2, [0] * 2))[-1][1]
    assert 0 <= msg["angle_min"] < msg["angle_increment"]
    assert abs(msg["angle_max"] - msg["angle_min"] - msg["angle_increment"] * 359) < 1e-4


def test_import_loads_no_native_code(monkeypatch):
    import importlib
    import sys

    import lidar_ld06 as L

    loaded = []

    class Spy:
        def __getattr__(self, name):
            loaded.append(name)
            raise AttributeError(name)

    monkeypatch.setitem(sys.modules, "fastpath", Spy())
    importlib.reload(L)
    assert L._yd_scan is None and loaded == []
    importlib.reload(L)                       # leave a clean module for other tests
