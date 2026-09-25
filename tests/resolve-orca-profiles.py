#!/usr/bin/env python3
"""Resolve the packaged Snapmaker U1 parents into one flat config for the Orca CLI,
which reads explicit JSON overrides and does not walk `inherits`.
Usage: resolve-orca-profiles.py <profiles-orca/Snapmaker dir> <out.json>
"""
import json
import pathlib
import sys

vendor = pathlib.Path(sys.argv[1])
selected = (
    ("machine", "Snapmaker U1 (0.4 nozzle)"),
    ("filament", "Snapmaker PLA @U1"),
    ("process", "0.20 Standard @Snapmaker U1 (0.4 nozzle)"),
)
merged = {}
for kind, name in selected:
    profiles = {}
    for path in (vendor / kind).rglob("*.json"):
        data = json.loads(path.read_text())
        key = data.get("name", path.stem)
        if key in profiles:
            raise ValueError(f"Duplicate {kind} profile: {key}")
        profiles[key] = data

    def resolve(key, ancestors=()):
        if key in ancestors:
            raise ValueError(f"Profile inheritance cycle: {key}")
        data = profiles[key]
        parent = data.get("inherits", "")
        result = resolve(parent, (*ancestors, key)) if parent else {}
        result.update(data)
        result.pop("inherits", None)
        return result

    merged.update(resolve(name))

assert merged["gcode_flavor"] == "klipper"
assert merged["nozzle_temperature"] == ["220"]
assert merged["layer_height"] == "0.2"
pathlib.Path(sys.argv[2]).write_text(json.dumps(merged))
