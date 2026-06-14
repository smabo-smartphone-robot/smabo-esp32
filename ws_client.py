"""WebSocket client for MicroPython.

Connects OUT to smabo-brain's /esp32 endpoint and auto-reconnects.
Exposes broadcast() so Robot is unchanged from the WSServer era.

Client→server frames MUST be masked (RFC 6455 §5.3);
server→client frames are unmasked.
"""

import binascii
import os

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio


def _mask_frame(payload, opcode=0x1):
    """Build a single masked client-to-server frame."""
    data = payload.encode() if isinstance(payload, str) else bytes(payload)
    n = len(data)
    mask = os.urandom(4)
    masked = bytes(data[i] ^ mask[i % 4] for i in range(n))

    frame = bytearray()
    frame.append(0x80 | opcode)   # FIN + opcode
    if n < 126:
        frame.append(0x80 | n)    # MASK bit + 7-bit length
    elif n < 65536:
        frame.append(0x80 | 126)
        frame.append((n >> 8) & 0xFF)
        frame.append(n & 0xFF)
    else:
        frame.append(0x80 | 127)
        for shift in range(56, -1, -8):
            frame.append((n >> shift) & 0xFF)
    frame.extend(mask)
    frame.extend(masked)
    return frame


class WSClient:
    """Persistent, auto-reconnecting WebSocket client.

    Drop-in replacement for WSServer: same broadcast() API so Robot is
    unchanged.  The ``on_message`` callback is called as
    ``on_message(None, text)`` — the first arg (unused client handle) is
    kept for API compatibility.
    """

    def __init__(self, host, port, path, on_message):
        self.host = host
        self.port = port
        self.path = path
        self.on_message = on_message   # callable(client, text)
        self._writer = None

    # ------------------------------------------------------------------ #
    # Public API (matches WSServer interface used by Robot)
    # ------------------------------------------------------------------ #
    def broadcast(self, text):
        """Send text to brain (mirrors WSServer.broadcast)."""
        self._send(text)

    def _send(self, text):
        if self._writer is None:
            return
        try:
            self._writer.write(_mask_frame(text))
            asyncio.create_task(self._drain())
        except Exception:
            self._writer = None

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
            print("WSClient: reconnect in 3 s …")
            await asyncio.sleep(3)

    async def _connect_and_run(self):
        reader, writer = await asyncio.open_connection(self.host, self.port)
        try:
            await self._handshake(reader, writer)
            self._writer = writer
            print("WSClient: connected to %s:%d%s" % (self.host, self.port, self.path))
            await self._read_loop(reader, writer)
        finally:
            self._writer = None
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

    async def _drain(self):
        try:
            if self._writer:
                await self._writer.drain()
        except Exception:
            self._writer = None

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
                writer.write(_mask_frame(b"", 0x8))
                await writer.drain()
                return
            elif opcode == 0x9:                     # ping → pong
                writer.write(_mask_frame(payload, 0xA))
                await writer.drain()
            elif opcode in (0x1, 0x2):              # text / binary
                try:
                    text = payload.decode()
                except Exception:
                    continue
                res = self.on_message(None, text)   # None = no per-client handle
                if hasattr(res, "__await__"):
                    await res
