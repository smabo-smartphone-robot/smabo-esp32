"""Pairing with smabo-brain.

A paired ESP32 shows its token as the first WebSocket frame
(``{"op": "auth", "token": ...}``). To pair, it shows a one-time invite code
from smabo-web instead (``{"op": "pair", "code": ...}``) and the brain answers
with a token and a relay key. The token goes to ``brain.token`` and only the
hash of the relay key to ``http.relay_key_sha256`` in config.json; from then
on the REST API answers only requests that carry the key in
``X-Smabo-Relay-Key`` (the brain adds it when it relays smabo-web's settings)
and sends no CORS headers.

The invite arrives as ``brain.invite`` (over Bluetooth during Wi-Fi setup, see
improv.py, or ``POST /config {"brain": {"invite": ...}}`` while not paired).
When the brain says it no longer knows the token (removed in smabo-web, or
the brain was reset), the ESP32 forgets the pairing so it can be invited again.

None of the secrets leave the board: config_json / the /config topic drop them.
"""

import binascii
import hashlib

RELAY_KEY_HEADER = "x-smabo-relay-key"     # header names are compared lower-cased
SECRET_KEYS = (("brain", "token"), ("brain", "invite"), ("http", "relay_key_sha256"))


def sha256_hex(text):
    return binascii.hexlify(hashlib.sha256(text.encode()).digest()).decode()


def same(a, b):
    """Compare two strings in time independent of where they differ."""
    if len(a) != len(b):
        return False
    diff = 0
    for x, y in zip(a, b):
        diff |= ord(x) ^ ord(y)
    return diff == 0


def without_secrets(config):
    """A copy of ``config`` without the token, invite and relay-key hash."""
    out = dict(config)
    for section, key in SECRET_KEYS:
        part = out.get(section)
        if isinstance(part, dict) and key in part:
            part = dict(part)
            del part[key]
            out[section] = part
    return out


def take_secrets(patch):
    """Split a REST patch: ``(patch without secrets, invite or None)``.

    The token and relay-key hash cannot be set over REST; an invite is taken
    out to be handled by :class:`Pairing`.
    """
    invite = None
    brain = patch.get("brain")
    if isinstance(brain, dict) and isinstance(brain.get("invite"), str) and brain["invite"].strip():
        invite = brain["invite"].strip()
    return without_secrets(patch), invite


class Pairing:
    """The ESP32's side of pairing, kept in the shared config."""

    def __init__(self, cfg, name="smabo-esp32"):
        self.cfg = cfg
        self.name = name
        self.refused = ""
        # False while the brain says it does not check tokens (it runs without
        # --pairing): then smabo-web talks to this REST directly, so it opens.
        self.brain_checks_tokens = True

    @property
    def token(self):
        return self.cfg.get("brain.token") or ""

    @property
    def locked(self):
        """Paired with a relay key and the brain checks tokens: REST only for the brain, no CORS."""
        return bool(self.cfg.get("http.relay_key_sha256")) and self.brain_checks_tokens

    def set_invite(self, code):
        self.cfg.update({"brain": {"invite": code}})
        self.cfg.save_now()

    def first_frame(self):
        """The frame to send right after connecting, or None (not paired, no invite)."""
        invite = self.cfg.get("brain.invite") or ""
        if invite:
            return {"op": "pair", "code": invite, "role": "esp32", "name": self.name}
        if self.token:
            return {"op": "auth", "token": self.token}
        return None

    def handle_reply(self, frame):
        """The brain's answer: True accepted, False refused, None not an answer."""
        op = frame.get("op")
        if op not in ("auth", "pair"):
            return None
        ok = frame.get("ok") is True
        reason = str(frame.get("reason") or "")
        if reason == "no_auth" or frame.get("auth") is False:
            # The brain does not check tokens: keep ours for when it does.
            self.brain_checks_tokens = False
            self.refused = ""
            if op == "pair":
                self.cfg.update({"brain": {"invite": ""}})
                self.cfg.save_now()
            return True
        self.brain_checks_tokens = True
        if op == "pair":
            if ok and frame.get("token"):
                key = frame.get("relay_key") or ""
                self.cfg.update({"brain": {"token": frame["token"], "invite": ""},
                                 "http": {"relay_key_sha256": sha256_hex(key) if key else ""}})
                print("Pairing: paired with the brain as", frame.get("name"))
            else:
                self.cfg.update({"brain": {"invite": ""}})       # one try per code
                print("Pairing: the brain refused the invite code (%s)" % reason)
            self.cfg.save_now()
        elif not ok:
            if self.token and reason == "unauthorized":
                self.cfg.update({"brain": {"token": ""}, "http": {"relay_key_sha256": ""}})
                self.cfg.save_now()
                print("Pairing: the brain no longer knows this ESP32; unpaired. Pair it again in smabo-web")
            elif not self.token:
                print("Pairing: the brain needs pairing. In smabo-web: Devices -> Add ESP32")
        self.refused = "" if ok else (reason or "refused")
        return ok

    def relay_allowed(self, key):
        """Whether a REST request may proceed (anything while not paired)."""
        want = self.cfg.get("http.relay_key_sha256") or ""
        if not want or not self.brain_checks_tokens:
            return True
        return bool(key) and same(sha256_hex(key), want)

    def status(self):
        st = {"paired": bool(self.token)}
        if self.refused:
            st["refused"] = self.refused
        return st
