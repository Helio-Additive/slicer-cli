"""Record or check the native plate maps, actual final nozzle and manual refusals.

Pass the CLI executable and --record to regenerate the vectors.
main.cpp 10430–10518 applies the selected plate, then ToolOrdering.cpp
3536–3614 selects the nozzle that Print.cpp 3241–3288 publishes.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "tests/fixtures/calib_base.3mf"
PROFILE = ROOT / "crates/libslic3r-rs/tests/data/cube_arachne_settings.json"
VECTORS = Path(__file__).parent / "data/plate_filament_routing.json"
VARIANTS = ["Direct Drive Standard", "Direct Drive High Flow"]
HARDWARE = {
    "nozzle_diameter": ["0.4", "0.6"],
    "nozzle_volume_type": ["Standard", "High Flow"],
    "physical_extruder_map": ["0", "1"],
    "extruder_type": ["Direct Drive", "Direct Drive"],
    "print_extruder_id": ["1", "2"],
    "printer_extruder_id": ["1", "2"],
    "filament_self_index": ["1", "1"],
    "extruder_variant_list": VARIANTS,
    "filament_extruder_variant": VARIANTS,
    "printer_extruder_variant": VARIANTS,
    "print_extruder_variant": VARIANTS,
}


TWO_MATERIALS = {
    "filament_colour": ["#FFFFFF", "#000000"],
    "filament_diameter": ["1.75", "1.75"],
    "filament_type": ["PLA", "PLA"],
    "filament_settings_id": ["Generic PLA", "Generic PLA"],
    "filament_ids": ["1", "2"],
    "filament_map": ["1", "2"],
    "filament_volume_map": ["0", "0"],
    "filament_nozzle_map": ["0", "1"],
    "filament_self_index": ["1", "2"],
    "filament_printable": ["3", "1"],
    "flush_multiplier": ["1", "1"],
    "flush_multiplier_fast": ["1", "1"],
    "flush_volumes_matrix": ["0", "280", "280", "0"] * 2,
}


def project(path, mode, slot, scope):
    with ZipFile(FIXTURE) as src, ZipFile(path, "w") as dst:
        for entry in src.infolist():
            data = src.read(entry)
            if entry.filename == "Metadata/project_settings.config":
                saved = json.loads(data)
                saved.update(HARDWARE, filament_printable=["3"], printer_model="Bambu Lab H2D",
                             printer_settings_id="Bambu Lab H2D 0.4 nozzle",
                             extruder_max_nozzle_count=["1", "1"], machine_start_gcode="; MARK_START")
                if scope:
                    saved.update(TWO_MATERIALS)
                data = json.dumps(saved).encode()
            elif entry.filename == "Metadata/model_settings.config":
                xml = data.decode()
                tag = '<metadata key="filament_map_mode" value="Auto For Flush"/>'
                assert tag in xml
                xml = xml.replace(tag, tag.replace("Auto For Flush", mode) if mode else "")
                if scope == "local-disables":
                    xml = xml.replace('<part id=', '<metadata key="enable_support" value="0"/><part id=', 1)
                if scope == "raft-override":
                    xml = xml.replace('<part id=', '<metadata key="raft_layers" value="0"/><part id=', 1)
                slot = "1 2" if scope else slot
                data = xml.replace('key="filament_maps" value="1"', f'key="filament_maps" value="{slot}"').encode()
            dst.writestr(entry, data)
        if scope == "range":
            dst.writestr("Metadata/layer_config_ranges.xml",
                         '<objects><object id="1"><range min_z="3" max_z="6">'
                         '<option opt_key="layer_height">0.2</option>'
                         '<option opt_key="sparse_infill_filament">2</option></range></object></objects>')


def slice_job(binary, base, directory, mode, plate_slot, profile_slot, mask, scope=None):
    directory.mkdir(exist_ok=True)
    model, profile, out = (directory / f for f in ("cube.3mf", "settings.json", "out.gcode"))
    project(model, mode, plate_slot, scope)
    settings = base.copy()
    settings.update(HARDWARE, filament_map_mode="Manual", filament_map=[str(profile_slot)],
                    filament_printable=[str(mask)], machine_start_gcode="; MARK_START",
                    filament_start_gcode=[""], change_filament_gcode="",
                    filament_end_gcode=["; ROUTE {current_extruder_id} {current_nozzle_id} "
                                        "{nozzle_diameter_at_nozzle_id[current_nozzle_id]} "
                                        "{nozzle_volume_types[current_nozzle_id]}"],
                    machine_end_gcode="; END")
    if scope:
        settings.update(TWO_MATERIALS, enable_prime_tower="0",
                        filament_start_gcode=["", ""], filament_end_gcode=["", ""])
        if scope in ("local-disables", "raft-override"):
            settings.update(enable_support="1", support_filament="2")
        elif scope == "raft":
            settings.update(raft_layers="2", support_interface_filament="2")
        elif scope == "named-printer":
            settings.update(wall_filament="2", printer_settings_id="Example dual nozzle")
        elif scope == "mixed":
            settings.update(filament_is_mixed=["1", "0"], filament_mixed_components=["2", ""],
                            filament_mixed_ratios=["100", ""])
    profile.write_text(json.dumps(settings))
    run = subprocess.run([binary, "--config", profile, "-o", out, model], cwd=directory,
                         capture_output=True, text=True)
    if run.returncode:
        errors = [line for line in run.stderr.splitlines() if line.startswith("Error:")]
        return {"status": run.returncode, "errors": errors}
    gcode = out.read_text()
    values = {}
    for key in ("filament_map_mode", "filament_map", "filament_map_2", "filament_nozzle_map",
                "filament_volume_map"):
        row = re.search(rf"^; {key} = (.*)$", gcode, re.M)
        values[key] = row[1] if row else None
    return {"status": 0, "maps": values, "terminal": re.findall(r"^; ROUTE .*$", gcode, re.M)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    cases = [(mode, slot, external, 3)
             for mode in ("Manual", "Auto For Flush", "Default", "Auto", None)
             for slot in (1, 2) for external in (1, 2)]
    cases.extend([("Manual", 2, 1, 1), ("Manual", 2, 2, 1),
                  ("Manual", 1, 2, 2), ("Auto For Flush", 1, 1, 2)])
    cases.extend(("Manual", 1, 1, 3, scope) for scope in
                 ("unused", "range", "local-disables", "raft-override", "raft", "named-printer", "mixed"))
    base = json.loads(PROFILE.read_text())
    actual = {}
    with tempfile.TemporaryDirectory() as tmp:
        for case in cases:
            label = "/".join(map(str, case))
            actual[label] = slice_job(args.binary.resolve(), base, Path(tmp) / label.replace("/", "-"), *case)
    if args.record:
        VECTORS.write_text("{\n" + ",\n".join(
            "  " + json.dumps(key) + ": " + json.dumps(value) for key, value in actual.items()
        ) + "\n}\n")
    else:
        expected = json.loads(VECTORS.read_text())
        mismatches = {key: [value, expected.get(key)] for key, value in actual.items() if value != expected.get(key)}
        assert actual == expected, json.dumps(mismatches, indent=2)


if __name__ == "__main__":
    main()
