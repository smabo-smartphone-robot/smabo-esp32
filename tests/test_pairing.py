"""Pairing on the ESP32: invite → token, REST locked to the brain, secrets kept in."""

import asyncio
import json

import pytest

import config as config_mod
from config import Config
from http_server import ConfigHTTPServer
from pairing import Pairing, same, sha256_hex, take_secrets, without_secrets
from robot import Robot
from ws_client import WSClient


@pytest.fixture
def cfg(tmp_path, monkeypatch):
    monkeypatch.setattr(config_mod, "_ticks_ms", lambda: 0)
    return Config(str(tmp_path / "config.json"))


def paired(cfg, token="tok", key="the-relay-key"):
    p = Pairing(cfg, "smabo")
    p.handle_reply({"op": "pair", "ok": True, "token": token, "relay_key": key, "name": "smabo"})
    return p


def test_invite_then_token(cfg):
    p = Pairing(cfg, "smabo")
    assert p.first_frame() is None and not p.locked
    p.set_invite("ABCD-EFGH")
    assert p.first_frame() == {"op": "pair", "code": "ABCD-EFGH", "role": "esp32", "name": "smabo"}
    assert p.handle_reply({"op": "pair", "ok": True, "token": "tok", "relay_key": "k"}) is True
    assert p.first_frame() == {"op": "auth", "token": "tok"}
    assert cfg.get("brain.invite") == "" and cfg.get("http.relay_key_sha256") == sha256_hex("k")
    # survives a reboot (saved at once)
    again = Config(cfg._path)
    assert again.get("brain.token") == "tok"


def test_refused_invite_is_dropped(cfg):
    p = Pairing(cfg, "smabo")
    p.set_invite("X")
    assert p.handle_reply({"op": "pair", "ok": False, "reason": "expired"}) is False
    assert p.first_frame() is None and p.status() == {"paired": False, "refused": "expired"}


def test_removed_esp32_unlocks(cfg):
    p = paired(cfg)
    assert p.locked
    p.handle_reply({"op": "auth", "ok": False, "reason": "unauthorized"})
    assert not p.locked and p.first_frame() is None and p.relay_allowed(None)


def test_relay_key_check(cfg):
    p = Pairing(cfg)
    assert p.relay_allowed(None)
    p = paired(cfg)
    assert p.relay_allowed("the-relay-key")
    assert not p.relay_allowed("nope") and not p.relay_allowed(None) and not p.relay_allowed("")


def test_same():
    assert same("abc", "abc") and not same("abc", "abd") and not same("abc", "ab")


def test_secrets_never_leave(cfg):
    paired(cfg)
    clean = without_secrets(cfg.data)
    assert "token" not in clean["brain"] and "relay_key_sha256" not in clean["http"]
    assert cfg.data["brain"]["token"] == "tok"        # the live config keeps them
    patch, invite = take_secrets({"brain": {"token": "evil", "invite": " C0DE ", "host": "h"},
                                  "http": {"relay_key_sha256": "x"}})
    assert patch == {"brain": {"host": "h"}, "http": {}} and invite == "C0DE"


class FakeWs:
    def __init__(self):
        self.sent, self.restarts = [], 0

    def broadcast(self, text):
        self.sent.append(json.loads(text))

    def restart(self):
        self.restarts += 1


def test_rest_config_hides_secrets_and_takes_an_invite(cfg):
    ws = FakeWs()
    p = Pairing(cfg, "smabo")
    robot = Robot(cfg, None, ws, p)
    robot.set_config({"brain": {"invite": "ABCD-EFGH"}})
    assert cfg.get("brain.invite") == "ABCD-EFGH" and ws.restarts == 1
    p.handle_reply({"op": "pair", "ok": True, "token": "tok", "relay_key": "k"})
    data = robot.config_json()
    assert data["brain"]["paired"] is True and "token" not in data["brain"]
    assert "relay_key_sha256" not in data["http"]
    # sending the shown config back changes nothing secret
    robot.set_config({"brain": data["brain"], "dc": {"max_linear": 0.21}})
    assert cfg.get("brain.token") == "tok" and cfg.get("dc.max_linear") == 0.21
    robot.sync_config_to_brain()
    snapshot = json.loads(ws.sent[-1]["msg"]["data"])
    assert "token" not in snapshot["brain"] and "relay_key_sha256" not in snapshot["http"]


# ── the REST server ────────────────────────────────────────────────────────
async def _request(port, method, path, headers=(), body=b""):
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    head = "%s %s HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n" % (method, path, len(body))
    head += "".join("%s: %s\r\n" % h for h in headers) + "\r\n"
    writer.write(head.encode() + body)
    await writer.drain()
    raw = await reader.read()
    writer.close()
    status = int(raw.split(b" ", 2)[1])
    headers_raw, _, payload = raw.partition(b"\r\n\r\n")
    return status, headers_raw.decode(), payload


def test_paired_rest_answers_only_the_brain(cfg):
    async def go():
        p = Pairing(cfg, "smabo")
        server = ConfigHTTPServer(Robot(cfg, None, FakeWs(), p), 0, p)
        srv = await asyncio.start_server(server._handle, "127.0.0.1", 0)
        port = srv.sockets[0].getsockname()[1]
        try:
            status, head, body = await _request(port, "GET", "/config")
            assert status == 200 and "Access-Control-Allow-Origin" in head
            assert json.loads(body)["brain"]["paired"] is False

            p.handle_reply({"op": "pair", "ok": True, "token": "tok", "relay_key": "k"})
            status, head, body = await _request(port, "GET", "/config")
            assert status == 401 and "Access-Control" not in head
            status, head, _ = await _request(port, "OPTIONS", "/config")
            assert status == 401
            status, head, body = await _request(port, "GET", "/config", [("X-Smabo-Relay-Key", "k")])
            assert status == 200 and "Access-Control" not in head
        finally:
            srv.close()
    asyncio.run(go())


# ── the WebSocket client against a stand-in brain ──────────────────────────
def _frame(text):
    data = text.encode()
    return bytes((0x81, len(data))) + data


async def _read_client_frame(reader):
    hdr = await reader.readexactly(2)
    n = hdr[1] & 0x7F
    if n == 126:
        n = int.from_bytes(await reader.readexactly(2), "big")
    mask = await reader.readexactly(4)
    data = await reader.readexactly(n)
    return json.loads(bytes(b ^ mask[i % 4] for i, b in enumerate(data)))


def test_ws_client_pairs_then_authenticates(cfg):
    async def go():
        seen, connected = [], asyncio.Event()

        async def brain(reader, writer):
            while (await reader.readline()) not in (b"\r\n", b""):
                pass
            writer.write(b"HTTP/1.1 101 Switching Protocols\r\n\r\n")
            first = await _read_client_frame(reader)
            seen.append(first)
            if first["op"] == "pair":
                writer.write(_frame(json.dumps({"op": "pair", "ok": True, "token": "tok", "relay_key": "k"})))
            else:
                writer.write(_frame(json.dumps({"op": "auth", "ok": first.get("token") == "tok"})))
                connected.set()
            await writer.drain()
            await asyncio.sleep(0.05)
            writer.close()

        srv = await asyncio.start_server(brain, "127.0.0.1", 0)
        port = srv.sockets[0].getsockname()[1]
        p = Pairing(cfg, "smabo")
        p.set_invite("ABCD-EFGH")
        on_connect = []
        ws = WSClient("127.0.0.1", port, "/", lambda c, t: None, lambda: on_connect.append(1), pairing=p)
        try:
            for _ in range(2):
                try:
                    await asyncio.wait_for(ws._connect_and_run(), 2)
                except OSError:
                    pass
        finally:
            srv.close()
        assert [f["op"] for f in seen] == ["pair", "auth"]
        assert cfg.get("brain.token") == "tok" and len(on_connect) == 2
    asyncio.run(go())
