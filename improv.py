"""Wi-Fi setup over Bluetooth LE (the Improv Wi-Fi protocol).

When the ESP32 cannot join Wi-Fi at boot (new robot, moved house, typo in the
password) it advertises the Improv service. smabo-web (Chrome / Edge, Web
Bluetooth) or any Improv client then sends the SSID and password; the ESP32
tries them, saves them on success and answers with its config URL.

Protocol: https://www.improv-wifi.com/ble/

Only started while Wi-Fi is down, and stopped shortly after provisioning, so a
robot that is already on the network does not accept new credentials over the
air. ``improv.enabled: false`` turns it off entirely.

The packet helpers at the top are plain Python (tested on the host); the BLE
part needs MicroPython's ``bluetooth`` module.
"""

try:
    import uasyncio as asyncio
except ImportError:
    import asyncio

# ── protocol constants ─────────────────────────────────────────────────────
_BASE = "00467768-6228-2272-4663-2774782680%s"
SERVICE_UUID = _BASE % "00"
STATE_UUID = _BASE % "01"
ERROR_UUID = _BASE % "02"
RPC_UUID = _BASE % "03"
RESULT_UUID = _BASE % "04"
CAPS_UUID = _BASE % "05"
SERVICE_DATA_UUID16 = 0x4677

STATE_AUTH_REQUIRED = 0x01
STATE_AUTHORIZED = 0x02
STATE_PROVISIONING = 0x03
STATE_PROVISIONED = 0x04

ERROR_NONE = 0x00
ERROR_INVALID_RPC = 0x01
ERROR_UNKNOWN_RPC = 0x02
ERROR_UNABLE_TO_CONNECT = 0x03
ERROR_NOT_AUTHORIZED = 0x04
ERROR_UNKNOWN = 0xFF

CMD_WIFI_SETTINGS = 0x01
CMD_IDENTIFY = 0x02

CAPS = 0x00  # bit 0 = can identify; the robot has no LED to blink


class ImprovError(Exception):
    """An RPC that cannot be handled; ``code`` is the Improv error state."""

    def __init__(self, code):
        super().__init__(code)
        self.code = code


def checksum(data):
    """Improv checksum: the sum of all bytes, low 8 bits."""
    return sum(data) & 0xFF


class RpcBuffer:
    """Collect RPC writes until a whole packet has arrived.

    A client may split one packet over several BLE writes. ``feed`` returns
    ``(command, payload)`` once the packet is complete and its checksum
    matches, ``None`` while more bytes are expected, and raises
    :class:`ImprovError` (and forgets the bytes) for a broken packet.
    """

    def __init__(self):
        self._buf = b""

    def feed(self, data):
        self._buf += bytes(data)
        buf = self._buf
        if len(buf) < 3 or len(buf) < buf[1] + 3:
            return None
        self._buf = b""
        end = buf[1] + 2
        if len(buf) > end + 1 or checksum(buf[:end]) != buf[end]:
            raise ImprovError(ERROR_INVALID_RPC)
        return buf[0], buf[2:end]


def parse_wifi_settings(payload):
    """``[len, ssid..., len, password...]`` → ``(ssid, password)`` as str."""
    try:
        n = payload[0]
        ssid = bytes(payload[1:1 + n])
        m = payload[1 + n]
        password = bytes(payload[2 + n:2 + n + m])
    except IndexError:
        raise ImprovError(ERROR_INVALID_RPC)
    if len(ssid) != n or len(password) != m or 2 + n + m != len(payload) or not ssid:
        raise ImprovError(ERROR_INVALID_RPC)
    try:
        return ssid.decode(), password.decode()
    except UnicodeError:
        raise ImprovError(ERROR_INVALID_RPC)


def rpc_result(command, strings=()):
    """Build an RPC result packet: ``[cmd, len, (len, bytes)*, checksum]``."""
    body = b""
    for s in strings:
        b = s.encode()
        body += bytes((len(b),)) + b
    packet = bytes((command, len(body))) + body
    return packet + bytes((checksum(packet),))


def adv_payload(state, name):
    """Advertising data and scan response for the Improv service.

    The 31-byte advertisement carries the flags, the 128-bit service UUID and
    the Improv service data (state, capabilities); the name goes in the scan
    response.
    """
    def field(kind, value):
        return bytes((len(value) + 1, kind)) + value

    uuid = bytes(reversed(bytes.fromhex(SERVICE_UUID.replace("-", ""))))
    service_data = bytes((SERVICE_DATA_UUID16 & 0xFF, SERVICE_DATA_UUID16 >> 8,
                          state, CAPS, 0, 0, 0, 0))
    adv = field(0x01, b"\x06") + field(0x07, uuid) + field(0x16, service_data)
    resp = field(0x09, name.encode()[:29])
    return adv, resp


# ── BLE ────────────────────────────────────────────────────────────────────
_IRQ_CENTRAL_CONNECT = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE = 3

_FLAG_READ = 0x0002
_FLAG_WRITE = 0x0008
_FLAG_NOTIFY = 0x0010

_CONNECT_TIMEOUT_MS = 20000
_STOP_AFTER_MS = 10000  # keep BLE up a little after success so the client reads the result


class ImprovBle:
    """The Improv GATT service, driven by an asyncio task.

    The BLE IRQ only records what arrived; :meth:`run` does the slow work
    (joining Wi-Fi) outside the IRQ.
    """

    def __init__(self, cfg, sta, name):
        import bluetooth

        self.cfg = cfg
        self.sta = sta
        self.name = name
        self._conns = set()
        self._rpc = RpcBuffer()
        self._pending = None      # (command, payload) or ImprovError
        self._state = STATE_AUTHORIZED

        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        U = bluetooth.UUID
        service = (U(SERVICE_UUID), (
            (U(STATE_UUID), _FLAG_READ | _FLAG_NOTIFY),
            (U(ERROR_UUID), _FLAG_READ | _FLAG_NOTIFY),
            (U(RPC_UUID), _FLAG_WRITE),
            (U(RESULT_UUID), _FLAG_READ | _FLAG_NOTIFY),
            (U(CAPS_UUID), _FLAG_READ),
        ))
        ((self._h_state, self._h_error, self._h_rpc, self._h_result, h_caps),) = \
            self._ble.gatts_register_services((service,))
        self._ble.gatts_set_buffer(self._h_rpc, 128)
        self._ble.gatts_set_buffer(self._h_result, 128)
        self._ble.gatts_write(h_caps, bytes((CAPS,)))
        self._ble.gatts_write(self._h_error, bytes((ERROR_NONE,)))
        self._set_state(STATE_AUTHORIZED)

    # -- BLE plumbing --------------------------------------------------------
    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self._conns.add(data[0])
        elif event == _IRQ_CENTRAL_DISCONNECT:
            self._conns.discard(data[0])
            self._advertise()
        elif event == _IRQ_GATTS_WRITE and data[1] == self._h_rpc:
            try:
                packet = self._rpc.feed(self._ble.gatts_read(self._h_rpc))
            except ImprovError as e:
                packet = e
            if packet is not None:
                self._pending = packet

    def _write(self, handle, value):
        self._ble.gatts_write(handle, value)
        for conn in self._conns:
            try:
                self._ble.gatts_notify(conn, handle)
            except OSError:
                pass

    def _set_state(self, state):
        self._state = state
        self._write(self._h_state, bytes((state,)))
        self._advertise()

    def _set_error(self, code):
        self._write(self._h_error, bytes((code,)))

    def _advertise(self):
        adv, resp = adv_payload(self._state, self.name)
        try:
            self._ble.gap_advertise(100000, adv_data=adv, resp_data=resp)
        except OSError as e:  # some stacks refuse while a central is connected
            print("Improv: advertise failed", e)

    def stop(self):
        self._ble.gap_advertise(None)
        self._ble.active(False)

    # -- protocol ----------------------------------------------------------
    async def _join(self, ssid, password):
        """Try the credentials; True once associated."""
        try:
            self.sta.disconnect()
        except OSError:
            pass
        self.sta.connect(ssid, password)
        for _ in range(_CONNECT_TIMEOUT_MS // 500):
            if self.sta.isconnected():
                return True
            await asyncio.sleep_ms(500)
        return False

    async def _handle(self, packet):
        if isinstance(packet, ImprovError):
            raise packet
        command, payload = packet
        if command == CMD_IDENTIFY:
            return False
        if command != CMD_WIFI_SETTINGS:
            raise ImprovError(ERROR_UNKNOWN_RPC)
        ssid, password = parse_wifi_settings(payload)
        print("Improv: trying Wi-Fi", ssid)
        self._set_error(ERROR_NONE)
        self._set_state(STATE_PROVISIONING)
        if not await self._join(ssid, password):
            self._set_state(STATE_AUTHORIZED)
            raise ImprovError(ERROR_UNABLE_TO_CONNECT)
        self.cfg.update({"wifi": {"ssid": ssid, "password": password}})
        self.cfg.save_now()
        port = self.cfg.get("http.port", 80)
        url = "http://%s%s/" % (self.sta.ifconfig()[0], "" if port == 80 else ":%d" % port)
        print("Improv: connected; config at", url)
        self._set_state(STATE_PROVISIONED)
        self._write(self._h_result, rpc_result(CMD_WIFI_SETTINGS, (url,)))
        return True

    async def run(self):
        """Serve RPCs until Wi-Fi is set up (or comes back by itself)."""
        print("Improv: Wi-Fi setup over Bluetooth as", self.name)
        while True:
            await asyncio.sleep_ms(100)
            if self.sta.isconnected() and self._pending is None and self._state != STATE_PROVISIONING:
                break  # the keepalive task got the old network back
            packet, self._pending = self._pending, None
            if packet is None:
                continue
            try:
                if await self._handle(packet):
                    await asyncio.sleep_ms(_STOP_AFTER_MS)
                    break
            except ImprovError as e:
                self._set_error(e.code)
        self.stop()
        print("Improv: stopped")


def start(cfg, sta):
    """Start Improv as a background task when Wi-Fi is down; returns the task or None."""
    if sta.isconnected() or not cfg.get("improv.enabled", True):
        return None
    try:
        improv = ImprovBle(cfg, sta, cfg.get("improv.name") or "smabo-" + cfg.get("wifi.hostname", "esp32"))
    except Exception as e:  # no bluetooth module, BLE busy, out of memory
        print("Improv: unavailable", e)
        return None
    return asyncio.create_task(improv.run())
