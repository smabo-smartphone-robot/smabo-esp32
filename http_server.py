"""Minimal async HTTP server exposing the config/mode REST API.

smabo-web reaches it through smabo-brain's /devices/esp32/... relay (or
directly with smabo-brain-ros, and for an invite before pairing):

    GET  /config           → full live config (JSON, wifi.password shown as "****")
    POST /config  {patch}  → deep-merge a config patch (may reboot)
    POST /mode    {modes}  → enable/disable subsystems

Real-time control (``/cmd_vel``, ``/servo/command``) and telemetry still
flow over the WebSocket relay via smabo-brain.

Pairing (pairing.py): until the ESP32 is paired, CORS is wide-open so the
browser app (served from a different origin) can call these endpoints, and
``{"brain": {"invite": ...}}`` pairs it. Once paired, a request without the
brain's relay key (``X-Smabo-Relay-Key``) gets 401 and there is no CORS.

The I/O style mirrors ws_client.py (manual reads, ``writer.aclose()``) for
MicroPython compatibility.
"""

import json

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio


from pairing import RELAY_KEY_HEADER

_STATUS = {200: "OK", 204: "No Content", 400: "Bad Request", 401: "Unauthorized", 404: "Not Found"}

_CORS = (
    "Access-Control-Allow-Origin: *\r\n"
    "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
    "Access-Control-Allow-Headers: Content-Type\r\n"
)


class ConfigHTTPServer:
    """Serves the config/mode REST API by delegating to a Robot instance."""

    def __init__(self, robot, port=80, pairing=None):
        """Store the orchestrator and listen port.

        Parameters
        ----------
        robot : robot.Robot
            Provides ``config_json()``, ``set_config()`` and ``set_mode()``.
        port : int, optional
            TCP port to listen on (default 80).
        """
        self.robot = robot
        self.port = port
        self.pairing = pairing
        self._server = None
        self._cors = True         # per request: off once paired

    async def run(self):
        """Bind the listener and keep the task alive (schedule once in amain)."""
        self._server = await asyncio.start_server(self._handle, "0.0.0.0", self.port)
        print("HTTP config API on :%d" % self.port)
        while True:
            await asyncio.sleep(3600)

    # ------------------------------------------------------------------ #
    # connection handling
    # ------------------------------------------------------------------ #
    async def _handle(self, reader, writer):
        """Parse one request, dispatch it, and close the connection."""
        try:
            req_line = await reader.readline()
            if not req_line:
                return
            parts = req_line.decode().split(" ")
            method = parts[0]
            path = parts[1] if len(parts) > 1 else "/"

            length = 0
            relay_key = None
            while True:
                line = await reader.readline()
                if not line or line == b"\r\n":
                    break
                text = line.decode()
                low = text.lower()
                if low.startswith("content-length:"):
                    try:
                        length = int(low.split(":", 1)[1].strip())
                    except ValueError:
                        length = 0
                elif low.startswith(RELAY_KEY_HEADER + ":"):
                    relay_key = text.split(":", 1)[1].strip()

            body = await self._read_exact(reader, length) if length else b""
            locked = self.pairing is not None and self.pairing.locked
            self._cors = not locked
            if locked and not self.pairing.relay_allowed(relay_key):
                await self._json(writer, 401, {"error": "relay_key",
                                               "message": "Paired: settings go through the brain"})
                return
            await self._route(writer, method, path, body)
        except Exception as e:
            print("http error:", e)
        finally:
            try:
                await writer.aclose()
            except Exception:
                pass

    async def _read_exact(self, reader, n):
        buf = b""
        while len(buf) < n:
            chunk = await reader.read(n - len(buf))
            if not chunk:
                break
            buf += chunk
        return buf

    # ------------------------------------------------------------------ #
    # routing
    # ------------------------------------------------------------------ #
    async def _route(self, writer, method, path, body):
        if method == "OPTIONS":           # CORS preflight
            await self._send(writer, 204, "")
            return

        if "?" in path:
            path = path.split("?", 1)[0]

        if path == "/config" and method == "GET":
            await self._json(writer, 200, self.robot.config_json())

        elif path == "/config" and method == "POST":
            patch = self._parse(body)
            if patch is None:
                await self._json(writer, 400, {"error": "bad json"})
            else:
                self.robot.set_config(patch)
                await self._json(writer, 200, {"ok": True})

        elif path == "/mode" and method == "POST":
            modes = self._parse(body)
            if modes is None:
                await self._json(writer, 400, {"error": "bad json"})
            else:
                self.robot.set_mode(modes)
                await self._json(writer, 200, {"ok": True})

        else:
            await self._json(writer, 404, {"error": "not found"})

    def _parse(self, body):
        """Return the decoded JSON body, ``{}`` if empty, or None if invalid."""
        if not body:
            return {}
        try:
            return json.loads(body.decode())
        except (ValueError, UnicodeError):
            return None

    # ------------------------------------------------------------------ #
    # responses
    # ------------------------------------------------------------------ #
    async def _json(self, writer, status, obj):
        await self._send(writer, status, json.dumps(obj), "application/json")

    async def _send(self, writer, status, payload, ctype="text/plain"):
        data = payload.encode() if isinstance(payload, str) else payload
        head = (
            "HTTP/1.1 %d %s\r\n"
            "%s"
            "Content-Type: %s\r\n"
            "Content-Length: %d\r\n"
            "Connection: close\r\n"
            "\r\n"
        ) % (status, _STATUS.get(status, "OK"), _CORS if self._cors else "", ctype, len(data))
        writer.write(head.encode())
        if data:
            writer.write(data)
        await writer.drain()
