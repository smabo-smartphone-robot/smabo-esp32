"""Improv Wi-Fi over BLE: packets, and the service driven by a fake BLE stack."""

import asyncio
import sys
import types

import pytest

import improv
from improv import (CMD_WIFI_SETTINGS, ERROR_INVALID_RPC, ERROR_UNABLE_TO_CONNECT,
                    ERROR_UNKNOWN_RPC, STATE_AUTHORIZED, STATE_PROVISIONED, ImprovError,
                    RpcBuffer, adv_payload, checksum, parse_wifi_settings, rpc_result)


def wifi_packet(ssid, password):
    s, p = ssid.encode(), password.encode()
    data = bytes((len(s),)) + s + bytes((len(p),)) + p
    body = bytes((CMD_WIFI_SETTINGS, len(data))) + data
    return body + bytes((checksum(body),))


# ── packets ────────────────────────────────────────────────────────────────
def test_wifi_settings_round_trip():
    cmd, payload = RpcBuffer().feed(wifi_packet("home", "pässword"))
    assert cmd == CMD_WIFI_SETTINGS
    assert parse_wifi_settings(payload) == ("home", "pässword")


def test_packet_split_over_writes():
    buf, pkt = RpcBuffer(), wifi_packet("home-network", "secret-password")
    assert buf.feed(pkt[:5]) is None
    assert buf.feed(pkt[5:20]) is None
    assert buf.feed(pkt[20:])[0] == CMD_WIFI_SETTINGS


def test_bad_checksum_is_rejected_and_forgotten():
    buf, pkt = RpcBuffer(), bytearray(wifi_packet("a", "b"))
    pkt[-1] ^= 1
    with pytest.raises(ImprovError) as e:
        buf.feed(pkt)
    assert e.value.code == ERROR_INVALID_RPC
    assert buf.feed(wifi_packet("a", "b"))[0] == CMD_WIFI_SETTINGS


@pytest.mark.parametrize("payload", [b"", b"\x05ab", b"\x00\x00", b"\x01a\x03b", b"\x01a\x00x"])
def test_malformed_wifi_settings(payload):
    with pytest.raises(ImprovError):
        parse_wifi_settings(payload)


def test_open_network_has_an_empty_password():
    assert parse_wifi_settings(b"\x04open\x00") == ("open", "")


def test_rpc_result_layout():
    r = rpc_result(CMD_WIFI_SETTINGS, ("http://1.2.3.4/",))
    assert r[0] == CMD_WIFI_SETTINGS and r[1] == len(r) - 3
    assert r[2] == 15 and r[3:18] == b"http://1.2.3.4/"
    assert r[-1] == checksum(r[:-1])


def test_advertisement_fits_and_carries_state():
    adv, resp = adv_payload(STATE_AUTHORIZED, "smabo-esp32-robot")
    assert len(adv) <= 31 and len(resp) <= 31
    assert bytes((0x77, 0x46, STATE_AUTHORIZED)) in adv
    # 128-bit UUID is little-endian on air
    assert adv[5:21] == bytes(reversed(bytes.fromhex(improv.SERVICE_UUID.replace("-", ""))))
    assert resp.endswith(b"smabo-esp32-robot")


# ── the service, with a fake bluetooth module ──────────────────────────────
class FakeBLE:
    def __init__(self):
        self.values, self.notified, self.adv, self.irq_handler = {}, [], None, None
        self._active = False

    def active(self, on=None):
        if on is not None:
            self._active = on
        return self._active

    def irq(self, h):
        self.irq_handler = h

    def gatts_register_services(self, services):
        ((_, chars),) = services
        return (tuple(range(10, 10 + len(chars))),)

    def gatts_set_buffer(self, h, n):
        pass

    def gatts_write(self, h, v):
        self.values[h] = bytes(v)

    def gatts_read(self, h):
        return self.values[h]

    def gatts_notify(self, conn, h):
        self.notified.append(h)

    def gap_advertise(self, interval, adv_data=None, resp_data=None):
        self.adv = adv_data


class FakeSta:
    def __init__(self, good_password):
        self.good, self.connected = good_password, False

    def isconnected(self):
        return self.connected

    def disconnect(self):
        self.connected = False

    def connect(self, ssid, password):
        self.connected = password == self.good

    def ifconfig(self):
        return ("192.168.1.42", "255.255.255.0", "192.168.1.1", "8.8.8.8")


class FakeCfg:
    def __init__(self):
        self.data, self.saved = {"http": {"port": 80}}, False

    def get(self, path, default=None):
        node = self.data
        for k in path.split("."):
            if not isinstance(node, dict) or k not in node:
                return default
            node = node[k]
        return node

    def update(self, partial):
        self.data.update(partial)

    def save_now(self):
        self.saved = True


@pytest.fixture
def ble(monkeypatch):
    fake = FakeBLE()
    mod = types.ModuleType("bluetooth")
    mod.BLE = lambda: fake
    mod.UUID = lambda s: s
    monkeypatch.setitem(sys.modules, "bluetooth", mod)
    monkeypatch.setattr(improv, "_CONNECT_TIMEOUT_MS", 1000)
    monkeypatch.setattr(improv, "_STOP_AFTER_MS", 0)
    monkeypatch.setattr(improv.asyncio, "sleep_ms", lambda ms: asyncio.sleep(0), raising=False)
    return fake


def send(service, fake, packet):
    fake.values[service._h_rpc] = packet
    fake.irq_handler(improv._IRQ_GATTS_WRITE, (0, service._h_rpc))


def test_service_provisions_saves_and_stops(ble):
    cfg, sta = FakeCfg(), FakeSta("right")
    service = improv.ImprovBle(cfg, sta, "smabo-test")
    ble.irq_handler(improv._IRQ_CENTRAL_CONNECT, (1, 0, b""))

    async def scenario():
        task = asyncio.ensure_future(service.run())
        send(service, ble, wifi_packet("home", "wrong"))
        for _ in range(50):
            await asyncio.sleep(0)
        assert ble.values[service._h_error] == bytes((ERROR_UNABLE_TO_CONNECT,))
        assert ble.values[service._h_state] == bytes((STATE_AUTHORIZED,))
        assert not cfg.saved

        send(service, ble, wifi_packet("home", "right"))
        await asyncio.wait_for(task, 1)

    asyncio.run(scenario())
    assert cfg.saved and cfg.data["wifi"] == {"ssid": "home", "password": "right"}
    assert ble.values[service._h_state] == bytes((STATE_PROVISIONED,))
    result = ble.values[service._h_result]
    assert result[3:3 + result[2]] == b"http://192.168.1.42/"
    assert service._h_result in ble.notified
    assert not ble.active()


def test_service_reports_unknown_and_broken_rpcs(ble):
    service = improv.ImprovBle(FakeCfg(), FakeSta("x"), "smabo-test")

    async def scenario():
        task = asyncio.ensure_future(service.run())
        body = bytes((0x7F, 0))
        send(service, ble, body + bytes((checksum(body),)))
        for _ in range(5):
            await asyncio.sleep(0)
        assert ble.values[service._h_error] == bytes((ERROR_UNKNOWN_RPC,))
        send(service, ble, b"\x01\x00\x00")
        for _ in range(5):
            await asyncio.sleep(0)
        assert ble.values[service._h_error] == bytes((ERROR_INVALID_RPC,))
        task.cancel()

    asyncio.run(scenario())


def test_start_only_when_wifi_is_down(ble):
    cfg = FakeCfg()
    up = FakeSta("x")
    up.connected = True
    assert improv.start(cfg, up) is None
    cfg.data["improv"] = {"enabled": False}
    assert improv.start(cfg, FakeSta("x")) is None
