"""Tiny asyncio WebSocket server (RFC 6455) for MicroPython.

No external dependencies.  Handles the HTTP upgrade handshake and text
frames (with the client->server mask), ping/pong and close.  Each connected
client gets a callback for inbound text messages, and the server can push
text to all clients.
"""

import hashlib
import binascii

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _accept_key(key):
    """Compute the Sec-WebSocket-Accept value for a client's key (RFC 6455).

    Parameters
    ----------
    key : str
        The client's ``Sec-WebSocket-Key`` header value.

    Returns
    -------
    str
        The base64-encoded accept token for the handshake response.
    """
    h = hashlib.sha1(key.encode() + _GUID)
    return binascii.b2a_base64(h.digest()).strip().decode()


def _build_frame(payload, opcode=0x1):
    """Encode a payload as a single unmasked server-to-client frame.

    Parameters
    ----------
    payload : str or bytes
        The frame payload; ``str`` is UTF-8 encoded.
    opcode : int, optional
        WebSocket opcode (default ``0x1`` = text).

    Returns
    -------
    bytearray
        The complete framed bytes ready to write to a socket.
    """
    data = payload.encode() if isinstance(payload, str) else payload
    n = len(data)
    frame = bytearray()
    frame.append(0x80 | opcode)  # FIN + opcode
    if n < 126:
        frame.append(n)
    elif n < 65536:
        frame.append(126)
        frame.append((n >> 8) & 0xFF)
        frame.append(n & 0xFF)
    else:
        frame.append(127)
        for shift in range(56, -1, -8):
            frame.append((n >> shift) & 0xFF)
    frame.extend(data)
    return frame


class WSServer:
    """A minimal asyncio WebSocket server that fans messages out to all clients."""

    def __init__(self, port, on_message):
        """Store the listen port and the inbound-message callback.

        Parameters
        ----------
        port : int
            TCP port to listen on.
        on_message : callable
            ``on_message(writer, text)`` invoked per inbound frame; may be
            synchronous or return an awaitable.
        """
        self.port = port
        self.on_message = on_message  # async or sync (client, text)
        self.clients = set()

    async def start(self):
        """Begin listening for connections on ``0.0.0.0:port``.

        Returns
        -------
        None
        """
        await asyncio.start_server(self._handle, "0.0.0.0", self.port)
        print("WebSocket server listening on :%d" % self.port)

    # -- broadcast ---------------------------------------------------------
    async def _drain(self, w):
        """Flush one client's write buffer, dropping the client on error.

        Parameters
        ----------
        w : asyncio.StreamWriter
            The client's writer.

        Returns
        -------
        None
        """
        try:
            await w.drain()
        except Exception:
            self.clients.discard(w)

    def broadcast(self, text):
        """Send text as a frame to every connected client.

        ``write`` only buffers the frame, so a drain task is scheduled per
        client to actually flush it to the socket.

        Parameters
        ----------
        text : str
            The text payload to broadcast.

        Returns
        -------
        None
        """
        frame = _build_frame(text, 0x1)
        for w in list(self.clients):
            try:
                w.write(frame)
                asyncio.create_task(self._drain(w))
            except Exception:
                self.clients.discard(w)

    # -- connection handling ----------------------------------------------
    async def _handle(self, reader, writer):
        """Handle one connection: handshake, register, read until close.

        Parameters
        ----------
        reader : asyncio.StreamReader
            The connection's read stream.
        writer : asyncio.StreamWriter
            The connection's write stream (used as the client handle).

        Returns
        -------
        None
        """
        try:
            if not await self._handshake(reader, writer):
                await writer.aclose()
                return
            self.clients.add(writer)
            await self._read_loop(reader, writer)
        except Exception as e:
            print("ws client error:", e)
        finally:
            self.clients.discard(writer)
            try:
                await writer.aclose()
            except Exception:
                pass

    async def _handshake(self, reader, writer):
        """Perform the HTTP Upgrade handshake.

        Parameters
        ----------
        reader : asyncio.StreamReader
            Stream to read the HTTP request headers from.
        writer : asyncio.StreamWriter
            Stream to write the 101 response to.

        Returns
        -------
        bool
            True if the handshake completed; False if no key was found.
        """
        key = None
        while True:
            line = await reader.readline()
            if not line or line == b"\r\n":
                break
            if line.lower().startswith(b"sec-websocket-key:"):
                key = line.split(b":", 1)[1].strip().decode()
        if not key:
            return False
        resp = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n\r\n" % _accept_key(key)
        )
        writer.write(resp.encode())
        await writer.drain()
        return True

    async def _read_exact(self, reader, n):
        """Read exactly ``n`` bytes from the stream.

        Parameters
        ----------
        reader : asyncio.StreamReader
            Stream to read from.
        n : int
            Number of bytes to read.

        Returns
        -------
        bytes
            Exactly ``n`` bytes.

        Raises
        ------
        OSError
            If the stream closes before ``n`` bytes are available.
        """
        buf = b""
        while len(buf) < n:
            chunk = await reader.read(n - len(buf))
            if not chunk:
                raise OSError("closed")
            buf += chunk
        return buf

    async def _read_loop(self, reader, writer):
        """Decode frames, answering ping/close and dispatching text/binary.

        Parameters
        ----------
        reader : asyncio.StreamReader
            Stream to read frames from.
        writer : asyncio.StreamWriter
            Stream used for pong/close replies and as the client handle.

        Returns
        -------
        None
            Returns when the peer sends a close frame.
        """
        while True:
            hdr = await self._read_exact(reader, 2)
            opcode = hdr[0] & 0x0F
            masked = hdr[1] & 0x80
            length = hdr[1] & 0x7F
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

            if opcode == 0x8:  # close
                writer.write(_build_frame(b"", 0x8))
                await writer.drain()
                return
            elif opcode == 0x9:  # ping -> pong
                writer.write(_build_frame(payload, 0xA))
                await writer.drain()
            elif opcode in (0x1, 0x2):  # text / binary
                try:
                    text = payload.decode()
                except Exception:
                    continue
                res = self.on_message(writer, text)
                if hasattr(res, "__await__"):
                    await res
