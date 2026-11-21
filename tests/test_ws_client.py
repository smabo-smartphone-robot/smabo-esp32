"""ws_client: frames are masked correctly and sent by one task, without piling up."""

import asyncio

import ws_client
from ws_client import WSClient, _apply_mask, _mask_frame


def test_mask_matches_bytewise_xor():
    mask = b"\x12\x34\xab\xcd"
    for n in (0, 1, 3, 4, 5, 125, 126, 4000):
        data = bytes((i * 7 + 3) & 0xFF for i in range(n))
        assert _apply_mask(bytearray(data), mask) == bytes(data[i] ^ mask[i % 4] for i in range(n))


def test_frame_unmasks_to_the_payload():
    for text in ("", "hi", "x" * 125, '{"a": 1}' * 500):
        f = _mask_frame(text)
        h = 2 if len(text) < 126 else 4
        mask = f[h:h + 4]
        body = bytes(b ^ mask[i % 4] for i, b in enumerate(f[h + 4:]))
        assert f[0] == 0x81 and body == text.encode()


def test_frame_header_lengths():
    assert _mask_frame("a" * 125)[1] == 0x80 | 125
    f = _mask_frame("a" * 4000)
    assert f[1] == 0x80 | 126 and (f[2] << 8 | f[3]) == 4000 and len(f) == 4 + 4 + 4000


class SlowWriter:
    """A socket that takes a while per drain, recording what was written."""

    def __init__(self):
        self.written = []
        self.draining = 0
        self.max_draining = 0

    def write(self, data):
        self.written.append(bytes(data))

    async def drain(self):
        self.draining += 1
        self.max_draining = max(self.max_draining, self.draining)
        await asyncio.sleep(0.01)
        self.draining -= 1


def test_one_sender_and_busy_link_drops_frames():
    async def go():
        ws = WSClient("h", 1, "/", lambda c, t: None)
        w = SlowWriter()
        ws._writer = w
        sender = asyncio.create_task(ws._send_loop(w))
        big = "x" * 4000
        for _ in range(20):          # a burst far beyond what the link takes
            ws.broadcast(big)
        assert ws.dropped > 0 and ws._queued <= ws_client._MAX_QUEUED
        for _ in range(100):
            await asyncio.sleep(0.01)
            if not ws._queue:
                break
        await asyncio.sleep(0.02)
        sender.cancel()
        assert w.max_draining == 1
        assert len(w.written) == 20 - ws.dropped
        # the link recovers: later frames go out again
        assert not ws._queue
    asyncio.run(go())


class StuckWriter(SlowWriter):
    async def drain(self):
        await asyncio.sleep(10)


def test_stuck_socket_ends_the_connection(monkeypatch):
    monkeypatch.setattr(ws_client, "_SEND_TIMEOUT_S", 0.05)

    async def go():
        ws = WSClient("h", 1, "/", lambda c, t: None)
        w = StuckWriter()
        ws._writer = w
        done = asyncio.Event()
        task = asyncio.create_task(ws._guard(ws._send_loop(w), done))
        ws.broadcast("hello")
        await asyncio.wait_for(done.wait(), 1)   # the sender gave up → reconnect
        await task
    asyncio.run(go())


def test_raw_json_publish_makes_one_valid_frame():
    import json

    from brain_link import BrainLink

    sent = []

    class FakeWs:
        def broadcast(self, payload, key=None):
            sent.append(_mask_frame(payload))

    body = bytearray(b'{"ranges":[1.0,0.0],"x":"' + b"y" * 300 + b'"}')
    BrainLink(FakeWs()).publish("/scan", memoryview(body)[:len(body)])
    f = sent[0]
    mask = f[4:8]
    text = bytes(b ^ mask[i % 4] for i, b in enumerate(f[8:]))
    frame = json.loads(text)
    assert frame["op"] == "publish" and frame["topic"] == "/esp32/scan"
    assert frame["msg"]["ranges"] == [1.0, 0.0]


def test_native_mask_is_used_only_if_it_matches(monkeypatch):
    import sys
    import types

    def good_mask(buf, start, n, m):
        _apply_mask(buf, m, start)

    monkeypatch.setitem(sys.modules, "fastpath", types.SimpleNamespace(mask=good_mask))
    assert ws_client._pick_mask() is not _apply_mask

    def bad_mask(buf, start, n, m):
        _apply_mask(buf, m, 0)                 # ignores start

    monkeypatch.setitem(sys.modules, "fastpath", types.SimpleNamespace(mask=bad_mask))
    assert ws_client._pick_mask() is _apply_mask


def test_keyed_frames_are_latest_only():
    ws = WSClient("h", 1, "/", lambda c, t: None)
    ws._writer = SlowWriter()
    ws.broadcast("s" * 2000, key="/scan")
    ws.broadcast("pong-1")
    ws.broadcast("t" * 2000, key="/scan")       # replaces the first scan
    ws.broadcast("pong-2")
    bodies = [len(f) for f in ws._queue]
    assert len(ws._queue) == 3 and ws.dropped == 0
    assert sum(bodies) == ws._queued
    assert bodies[0] < 100 and bodies[1] > 2000 and bodies[2] < 100   # pong-1, newest scan, pong-2


def test_sent_keyed_frame_is_forgotten():
    async def go():
        ws = WSClient("h", 1, "/", lambda c, t: None)
        w = SlowWriter()
        ws._writer = w
        sender = asyncio.create_task(ws._send_loop(w))
        ws.broadcast("s" * 100, key="/scan")
        await asyncio.sleep(0.03)
        assert not ws._latest and not ws._queue
        ws.broadcast("t" * 100, key="/scan")       # queued, nothing to replace
        await asyncio.sleep(0.03)
        sender.cancel()
        assert len(w.written) == 2
    asyncio.run(go())
