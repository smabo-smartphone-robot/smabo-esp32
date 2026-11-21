"""WebSocket client for MicroPython.

Connects OUT to the brain (ws://host:port/) and auto-reconnects. With a
pairing.Pairing, the first frame is the token or an invite code (pairing.py).
Exposes broadcast() so Robot is unchanged from the WSServer era.

Client→server frames MUST be masked (RFC 6455 §5.3);
server→client frames are unmasked.

Outgoing frames go through one queue that a single sender task writes out, so
only one drain() ever waits on the socket. When the link is slower than what
we publish (e.g. /scan over weak Wi-Fi), new frames are dropped once
_MAX_QUEUED bytes are waiting, instead of piling up. A send error ends the
connection and the client reconnects. Frames sent with a ``key`` (e.g. /scan)
are "latest only": a newer one replaces the one still waiting, so a stream of
big frames never holds more than one of them ahead of pongs and the rest.
"""

import binascii
import json
import os

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

# bytes allowed to wait for the socket; newer frames are dropped beyond this
# (about two /scan frames: the ESP32 has little RAM to spare)
_MAX_QUEUED = 6144

# a socket that takes longer than this to accept a frame is treated as dead
# and the connection is rebuilt
_SEND_TIMEOUT_S = 5


def _mask_frame(payload, opcode=0x1):
    """Build a single masked client-to-server frame (one allocation).

    ``payload`` is text, bytes, or a tuple of byte chunks sent as one message.
    """
    if isinstance(payload, str):
        payload = payload.encode()
    parts = payload if isinstance(payload, tuple) else (payload,)
    n = 0
    for part in parts:
        n += len(part)
    if n < 126:
        head = bytes((0x80 | opcode, 0x80 | n))          # FIN + opcode, MASK + 7-bit length
    elif n < 65536:
        head = bytes((0x80 | opcode, 0x80 | 126, (n >> 8) & 0xFF, n & 0xFF))
    else:
        head = bytes([0x80 | opcode, 0x80 | 127] + [(n >> s) & 0xFF for s in range(56, -1, -8)])
    mask = os.urandom(4)
    h = len(head) + 4
    frame = bytearray(h + n)
    frame[:len(head)] = head
    frame[len(head):h] = mask
    p = h
    for part in parts:
        frame[p:p + len(part)] = part
        p += len(part)
    payload = parts = None                               # free the text before masking
    global _mask
    if _mask is None:
        _mask = _pick_mask()
    _mask(frame, mask, h)
    return frame


def _apply_mask(buf, mask, start=0):
    """XOR ``buf[start:]`` in place with the repeating 4-byte ``mask``."""
    m0, m1, m2, m3 = mask[0], mask[1], mask[2], mask[3]
    n = len(buf)
    i = start
    while i + 4 <= n:
        buf[i] ^= m0
        buf[i + 1] ^= m1
        buf[i + 2] ^= m2
        buf[i + 3] ^= m3
        i += 4
    k = 0
    while i < n:
        buf[i] ^= mask[k]
        i += 1
        k += 1
    return buf


def _pick_mask():
    """fastpath.mask (native code) if it loads and matches _apply_mask, else _apply_mask."""
    try:
        import fastpath
        m = b"\x12\x34\xab\xcd"
        a = bytearray(range(7, 7 + 23))
        b = bytearray(a)
        fastpath.mask(a, 2, len(a), m)
        _apply_mask(b, m, 2)
        if a == b:
            return lambda buf, mask, start=0: fastpath.mask(buf, start, len(buf), mask)
    except Exception:
        pass
    return _apply_mask


_mask = None


class WSClient:
    """Persistent, auto-reconnecting WebSocket client.

    Drop-in replacement for WSServer: same broadcast() API so Robot is
    unchanged.  The ``on_message`` callback is called as
    ``on_message(None, text)`` — the first arg (unused client handle) is
    kept for API compatibility.
    """

    def __init__(self, host, port, path, on_message, on_connect=None, pairing=None):
        self.host = host
        self.port = port
        self.path = path
        self.on_message = on_message   # callable(client, text)
        self.on_connect = on_connect   # optional callable() run after each connect
        # pairing.Pairing: the first frame (token / invite) and the brain's answer
        self.pairing = pairing
        self._writer = None
        self._refused = False
        self._queue = []          # masked frames waiting for the sender task
        self._queued = 0          # their total size in bytes
        self._latest = {}         # key -> its frame still waiting in _queue
        self._wake = asyncio.Event()
        self.dropped = 0          # frames dropped because the link was busy

    def restart(self):
        """Drop the connection so the next attempt uses new settings (e.g. an invite)."""
        w = self._writer
        self._writer = None
        if w is not None:
            try:
                w.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ #
    # Public API (matches WSServer interface used by Robot)
    # ------------------------------------------------------------------ #
    def broadcast(self, text, key=None):
        """Send text to brain (mirrors WSServer.broadcast).

        With ``key``, only the newest frame of that key waits to be sent: one
        still in the queue is replaced (used for /scan).
        """
        self._send(text, key)

    def _send(self, text, key=None):
        if self._writer is None:
            return
        if key is not None:
            old = self._latest.pop(key, None)
            if old is not None:
                q = self._queue
                for idx in range(len(q)):
                    if q[idx] is old:
                        del q[idx]
                        self._queued -= len(old)
                        break
                old = None
        # check the room first, so a busy link costs no frame building
        size = sum(len(p) for p in text) if isinstance(text, tuple) else len(text)
        if self._queued + size + 14 > _MAX_QUEUED:
            self._drop()
            return
        try:
            frame = _mask_frame(text)
        except MemoryError:
            self._drop()
            return
        if self._enqueue(frame) and key is not None:
            self._latest[key] = frame

    def _drop(self):
        self.dropped += 1
        if self.dropped == 1 or self.dropped % 100 == 0:
            print("WSClient: link busy, dropped %d frames" % self.dropped)

    def _enqueue(self, frame, force=False):
        """Queue a frame for the sender task (dropped if the link is backed up)."""
        if not force and self._queued + len(frame) > _MAX_QUEUED:
            self._drop()
            return False
        self._queue.append(frame)
        self._queued += len(frame)
        self._wake.set()
        return True

    async def _send_loop(self, writer):
        """The only task that writes to the socket after the HTTP upgrade."""
        while True:
            while not self._queue:
                self._wake.clear()
                await self._wake.wait()
            frame = self._queue.pop(0)
            self._queued -= len(frame)
            if self._latest:
                for k in self._latest:
                    if self._latest[k] is frame:
                        del self._latest[k]
                        break
            writer.write(frame)
            frame = None
            await asyncio.wait_for(writer.drain(), _SEND_TIMEOUT_S)

    async def _guard(self, coro, done):
        """Run a connection task; whichever ends first ends the connection."""
        try:
            await coro
        except Exception as e:
            print("WSClient error:", e)
        finally:
            done.set()

    # ------------------------------------------------------------------ #
    # Connection lifecycle
    # ------------------------------------------------------------------ #
    async def run(self):
        """Connect and stay connected indefinitely (schedule once in amain)."""
        while True:
            try:
                await self._connect_and_run()
            except Exception as e:
                print("WSClient error:", e)
            self._writer = None
            # A refused token stays refused until someone pairs this ESP32 again
            delay = 15 if self._refused else 3
            print("WSClient: reconnect in %d s …" % delay)
            await asyncio.sleep(delay)

    async def _connect_and_run(self):
        reader, writer = await asyncio.open_connection(self.host, self.port)
        self._queue = []
        self._queued = 0
        self._latest = {}
        done = asyncio.Event()
        tasks = [asyncio.create_task(self._guard(self._send_loop(writer), done))]
        try:
            await self._handshake(reader, writer)
            self._writer = writer
            self._refused = False
            if not await self._authenticate(reader, writer):
                self._refused = True
                return
            print("WSClient: connected to %s:%d%s" % (self.host, self.port, self.path))
            if self.on_connect is not None:
                try:
                    res = self.on_connect()
                    if hasattr(res, "__await__"):
                        await res
                except Exception as e:
                    print("WSClient on_connect error:", e)
            tasks.append(asyncio.create_task(self._guard(self._read_loop(reader, writer), done)))
            await done.wait()
        finally:
            self._writer = None
            for t in tasks:
                t.cancel()
            try:
                await writer.aclose()
            except Exception:
                pass

    async def _handshake(self, reader, writer):
        key = binascii.b2a_base64(os.urandom(16)).strip().decode()
        writer.write((
            "GET %s HTTP/1.1\r\n"
            "Host: %s:%d\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Key: %s\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
            % (self.path, self.host, self.port, key)
        ).encode())
        await writer.drain()
        # Discard response headers
        while True:
            line = await reader.readline()
            if not line or line == b"\r\n":
                break

    def _auth_reply(self, text):
        """Hand an auth / pair answer to the pairing; True if it was one."""
        if self.pairing is None or ('"auth"' not in text and '"pair"' not in text):
            return None
        try:
            frame = json.loads(text)
        except ValueError:
            return None
        if not isinstance(frame, dict):
            return None
        return self.pairing.handle_reply(frame)

    async def _authenticate(self, reader, writer):
        """Send the token or invite and wait for the answer.

        A brain without pairing (the default) says so at once; smabo-brain-ros
        does not answer, and after 2 s we go on as before. False when the brain refused us.
        """
        first = self.pairing.first_frame() if self.pairing is not None else None
        if first is None:
            return True
        self._enqueue(_mask_frame(json.dumps(first)), force=True)
        try:
            opcode, payload = await asyncio.wait_for(self._read_frame(reader, writer), 2)
        except asyncio.TimeoutError:
            return True
        if opcode is None:
            return False
        try:
            text = payload.decode()
        except Exception:
            return True
        ok = self._auth_reply(text)
        if ok is None:                      # an old brain already talking
            res = self.on_message(None, text)
            if hasattr(res, "__await__"):
                await res
            return True
        return ok

    async def _read_exact(self, reader, n):
        buf = b""
        while len(buf) < n:
            chunk = await reader.read(n - len(buf))
            if not chunk:
                raise OSError("connection closed")
            buf += chunk
        return buf

    async def _read_loop(self, reader, writer):
        while True:
            opcode, payload = await self._read_frame(reader, writer)
            if opcode is None:
                return
            try:
                text = payload.decode()
            except Exception:
                continue
            if self._auth_reply(text) is not None:
                continue
            res = self.on_message(None, text)   # None = no per-client handle
            if hasattr(res, "__await__"):
                await res

    async def _read_frame(self, reader, writer):
        """The next text / binary frame as ``(opcode, payload)``; ``(None, b"")`` on close.

        Pings are answered here.
        """
        while True:
            hdr = await self._read_exact(reader, 2)
            opcode = hdr[0] & 0x0F
            masked  = hdr[1] & 0x80   # server→client is normally unmasked
            length  = hdr[1] & 0x7F
            if length == 126:
                ext = await self._read_exact(reader, 2)
                length = (ext[0] << 8) | ext[1]
            elif length == 127:
                ext = await self._read_exact(reader, 8)
                length = 0
                for b in ext:
                    length = (length << 8) | b
            mask = await self._read_exact(reader, 4) if masked else b"\0\0\0\0"
            payload = await self._read_exact(reader, length) if length else b""
            if masked:
                payload = bytes(payload[i] ^ mask[i % 4] for i in range(length))

            if opcode == 0x8:                       # close
                self._enqueue(_mask_frame(b"", 0x8), force=True)
                await asyncio.sleep(0.05)           # let the sender flush it
                return None, b""
            elif opcode == 0x9:                     # ping → pong
                self._enqueue(_mask_frame(payload, 0xA), force=True)
            elif opcode in (0x1, 0x2):              # text / binary
                return opcode, payload
