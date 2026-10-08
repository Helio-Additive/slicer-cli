#!/usr/bin/env python3
"""check_preset_gcode.py VENDOR_DIR KIND NAME GCODE...

The shipped preset of KIND/NAME in VENDOR_DIR holds only its differences from
the presets it `inherits`. A settings file read as given therefore leaves every
key its parents define at the engine's default, while the desktop reads the
preset over its parents. This checks the other half of that: for every key the
preset's OWN file does not state and its parents do, the G-code header must
carry the parent's flattened value.

    VENDOR_DIR  a vendor directory of a profiles tree (…/profiles/BBL,
                …/profiles-orca/Snapmaker)
    KIND        machine | process | filament
    NAME        the preset's own "name", or the path of a settings file (a user
                preset, an export): the file's own keys, over the chain of the
                preset it names in "inherits"
    GCODE...    one or more sliced G-code files' headers to check

Exit status is non-zero with the mismatches listed.
"""
import json
import pathlib
import re
import sys

META = ("name", "inherits", "include", "from", "type", "instantiation",
        "setting_id", "filament_id", "version", "description", "url")

# The keys the input-path audit measured, one list per kind (input-path-audit.md,
# ~/audit-inputs/lib.sh: KM, KP, KF). They are the settings a hybrid gets wrong
# and a reader of the parent chain gets right; the rest of a preset is either
# the engine's own (it adapts accelerations and G-code to the nozzle, turns the
# prime tower off for one filament) or text this header escapes its own way.
AUDIT_KEYS = {
    "machine": ("printer_settings_id", "printer_model", "nozzle_diameter", "printable_area",
                "printable_height", "retraction_length", "z_hop", "extruder_clearance_height_to_rod",
                "machine_max_speed_x", "gcode_flavor", "single_extruder_multi_material", "printer_agent",
                "bed_mesh_max", "bed_mesh_min", "scan_first_layer", "bed_exclude_area"),
    "process": ("print_settings_id", "layer_height", "wall_loops", "sparse_infill_density", "top_shell_layers",
                "outer_wall_speed", "sparse_infill_speed", "initial_layer_speed", "line_width", "bridge_speed",
                "outer_wall_acceleration", "default_acceleration"),
    "filament": ("filament_settings_id", "filament_type", "nozzle_temperature",
                 "nozzle_temperature_initial_layer", "filament_max_volumetric_speed", "filament_density",
                 "filament_flow_ratio", "additional_cooling_fan_speed", "hot_plate_temp"),
}


def load(kind):
    out = {}
    for path in (vendor / kind).rglob("*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        out[data.get("name", path.stem)] = data
    return out


def flatten(kind, name, seen=()):
    data = profiles[kind][name]
    parent = data.get("inherits", "")
    if parent and parent not in seen:
        result = flatten(kind, parent, (*seen, name))
    else:
        result = {}
    for key in data.get("include") or []:
        result.update({k: v for k, v in flatten(kind, key, (*seen, name)).items() if k not in META})
    result.update(data)
    result.pop("inherits", None)
    return result


def header(path):
    text = pathlib.Path(path).read_text(encoding="utf-8", errors="replace")
    out = {}
    for match in re.finditer(r"^; ([a-z0-9_]+) = (.*)$", text, re.M):
        out.setdefault(match.group(1), match.group(2).strip())
    return out


def first(value):
    return re.sub(r"\"", "", str(value[0] if isinstance(value, list) else value))


def same_number(a, b):
    try:
        return abs(float(a) - float(b)) < 1e-9
    except ValueError:
        return False


POINT = re.compile(r"^(-?[\d.]+)x(-?[\d.]+)$")


def norm(text):
    """A header or preset value with its punctuation flattened: "100%" -> "100",
    quotes off, and a point "0x180" -> "0,180" (the header writes a point list
    as XxY entries too, so the split below stays right)."""
    text = text.strip().strip('"')
    if text.endswith("%"):
        text = text[:-1]
    match = POINT.match(text)
    return match.group(1) + "," + match.group(2) if match else text


def matches(header_value, flat_value):
    """True when the G-code's value for a key is the flattened preset's, None
    when the two cannot be compared.

    A scalar compares equal, numbers as numbers, a percentage as its number. A
    G-code blob or a quoted list is left out: the header escapes those its own
    way. A list compares entry by entry; a list whose entries are all the same
    also compares against the value repeated (the engine writes one entry per
    extruder: the U1's two-value bed_mesh_max reaches the header as 267,267).
    """
    if isinstance(flat_value, list):
        want = [norm(first(v)) for v in flat_value]
        if not want or any(w == "" for w in want):
            return None
        got = [norm(part) for part in header_value.split(",")]
        if len(set(want)) == 1:
            return all(g == want[0] or same_number(g, want[0]) for g in got)
        if len(got) != len(want):
            return None
        return all(g == w or same_number(g, w) for g, w in zip(got, want))
    text = norm(first(flat_value))
    if text == "" or any(c in text for c in "\n{}\""):
        return None
    got = norm(header_value.split(";")[0])
    return got == text or same_number(got, text)


vendor, kind, name = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
profiles = {kind: load(kind)}
leaf = pathlib.Path(name)
if leaf.suffix == ".json" and leaf.exists():
    # A settings file of its own (a user preset, a desktop export): its keys are
    # the leaf, and the preset it inherits is the chain.
    data = json.loads(leaf.read_text(encoding="utf-8"))
    own = {k: v for k, v in data.items() if k not in META}
    whole = flatten(kind, data["inherits"]) if data.get("inherits") else {}
    whole.update(own)
    name = str(leaf)
else:
    own = {k: v for k, v in profiles[kind][name].items() if k not in META}
    whole = flatten(kind, name)
from_parents = sorted(k for k in whole if k not in own and k not in META)
audit_keys = set(AUDIT_KEYS.get(kind, ()))
bad, checked, skipped = [], [], []
for gcode in sys.argv[4:]:
    got = header(gcode)
    for key in from_parents:
        if key not in got or key not in audit_keys:
            continue
        want = matches(got[key], whole[key])
        if want is None:
            skipped.append(key)
            continue
        checked.append("%s=%s" % (key, first(whole[key])))
        if not want:
            bad.append("%s: G-code %r, the parent value %r" % (key, got[key], first(whole[key])))
if not checked:
    print("no key the parents define appears in the G-code header; the check proves nothing")
    sys.exit(2)
print("checked %d of the audit's %d key(s) for this kind, over %d file(s), %d mismatched (%s)"
      % (len(checked), len(audit_keys), len(sys.argv) - 4, len(bad), ", ".join(sorted(set(checked)))))
if skipped:
    print("left out %d key(s) that cannot be compared: %s" % (len(skipped), ", ".join(sorted(set(skipped)))))
for line in bad:
    print("MISMATCH: " + line)
sys.exit(1 if bad else 0)
