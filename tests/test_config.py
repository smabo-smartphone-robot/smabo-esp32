"""Config: defaults merged under the stored file, dotted access, saves."""

import json

import config
from config import Config, DEFAULTS, _deep_merge


def test_deep_merge_overrides_nested_and_adds_new_keys():
    base = {"dc": {"max_linear": 0.3, "pins": {"stby": 1}}, "name": "a"}
    out = _deep_merge(base, {"dc": {"max_linear": 0.5}, "extra": {"x": 1}})
    assert out == {"dc": {"max_linear": 0.5, "pins": {"stby": 1}}, "name": "a", "extra": {"x": 1}}
    assert base["dc"]["max_linear"] == 0.3  # inputs untouched


def test_deep_merge_replaces_a_dict_with_a_scalar():
    assert _deep_merge({"a": {"b": 1}}, {"a": 2}) == {"a": 2}


def test_missing_or_broken_file_gives_defaults(tmp_path):
    assert Config(str(tmp_path / "none.json")).data == DEFAULTS
    bad = tmp_path / "bad.json"
    bad.write_text("{")
    assert Config(str(bad)).data == DEFAULTS


def test_stored_values_override_defaults(tmp_path):
    p = tmp_path / "c.json"
    p.write_text(json.dumps({"dc": {"max_linear": 0.9}}))
    c = Config(str(p))
    assert c.get("dc.max_linear") == 0.9
    assert c.get("dc.max_angular") == DEFAULTS["dc"]["max_angular"]


def test_get_returns_default_for_missing_paths(tmp_path):
    c = Config(str(tmp_path / "c.json"))
    assert c.get("dc.nope", 7) == 7
    assert c.get("dc.max_linear.deeper", "x") == "x"


def test_update_marks_dirty_and_save_now_writes(tmp_path, monkeypatch):
    p = tmp_path / "c.json"
    monkeypatch.setattr(config, "_ticks_ms", lambda: 100)
    c = Config(str(p))
    c.update({"dc": {"max_linear": 0.4}})
    assert c._dirty and c._save_at == 100 + config._DEBOUNCE_MS
    assert not p.exists()
    c.save_now()
    assert not c._dirty
    assert json.loads(p.read_text())["dc"]["max_linear"] == 0.4
    assert Config(str(p)).get("dc.max_linear") == 0.4
