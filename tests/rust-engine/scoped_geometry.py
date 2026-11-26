"""Compare wall and sparse deposition by part and height against the Bambu CLI.

Regenerate the vectors with scoped_geometry.py <native slicer_cli> --record.
PrintApply.cpp 973-1185 assigns regions; PrintObjectSlice.cpp 252-526 splits
the volume slices into those regions before perimeters and fill are made.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import re
import subprocess
import tempfile
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[2]
VECTORS = Path(__file__).parent / "data/scoped_geometry.json"
FIXTURE = ROOT / "tests/fixtures/calib_base.3mf"
PROFILE = ROOT / "crates/libslic3r-rs/tests/data/cube_arachne_settings.json"
# The build item centers the first cube at X128. The second is 36 mm away.
CASES = {
    "base": None,
    "three-five": ("parts", "wall_loops", "3", "5"),
    "five-three": ("parts", "wall_loops", "5", "3"),
    "three-three": ("parts", "wall_loops", "3", "3"),
    "five-in-band": ("range", "wall_loops", "5"),
    "hollow-left": ("parts", "sparse_infill_density", "0%", "30%"),
    "hollow-right": ("parts", "sparse_infill_density", "30%", "0%"),
    "hollow-in-band": ("range", "sparse_infill_density", "0%"),
}


def make_project(path, case):
    with ZipFile(FIXTURE) as source, ZipFile(path, "w") as dest:
        for info in source.infolist():
            data = source.read(info)
            if case and case[0] == "parts":
                if info.filename == "3D/3dmodel.model":
                    mesh = re.search(rb'<object id="1".*?</object>', data, re.S)[0]
                    data = data.replace(mesh, mesh + mesh.replace(b'id="1"', b'id="3"', 1))
                    item = re.search(rb'<component objectid="1".*?/>', data)[0]
                    second = b'<component objectid="3" transform="1 0 0 0 1 0 0 0 1 36 0 0"/>'
                    data = data.replace(item, item + second)
                elif info.filename == "Metadata/model_settings.config":
                    _, key, left, right = case
                    part = re.search(rb'<part id="1".*?</part>', data, re.S)[0]
                    parts = []
                    for volume, value in [(1, left), (3, right)]:
                        metadata = f'<metadata key="{key}" value="{value}"/>'.encode()
                        copy = part.replace(b'id="1"', f'id="{volume}"'.encode(), 1)
                        parts.append(copy.replace(b"</part>", metadata + b"</part>"))
                    data = data.replace(part, b"".join(parts))
            dest.writestr(info, data)
        if case and case[0] == "range":
            _, key, value = case
            dest.writestr("Metadata/layer_config_ranges.xml",
                          '<objects><object id="1"><range min_z="3" max_z="6">'
                          '<option opt_key="layer_height">0.2</option>'
                          f'<option opt_key="{key}">{value}</option></range></object></objects>')


def deposition(path):
    counts = Counter()
    height, feature, x = 0., "", 0.
    with path.open() as stream:
        for line in stream:
            if line.startswith("; Z_HEIGHT:"):
                height = float(line.split(":")[1])
            elif line.startswith("; FEATURE:"):
                feature = line.partition(":")[2].strip()
            elif line.startswith(("G0 ", "G1 ", "G2 ", "G3 ")):
                fields = {word[0]: word[1:] for word in line.split(";")[0].split()[1:]}
                if "X" in fields:
                    x = float(fields["X"])
                if ("X" in fields or "Y" in fields) and "E" in fields and float(fields["E"]) > 0:
                    counts[height, feature, x > 146.] += 1
    return {f"{z}mm": {role: [counts[z, role, right] for right in (False, True)]
                        for role in ("Inner wall", "Sparse infill")} for z in (2, 4, 8)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("binary", type=Path)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args()
    binary = args.binary.resolve()
    expected = {} if args.record else json.loads(VECTORS.read_text())
    actual = {}
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        settings = json.loads(PROFILE.read_text())
        settings["wall_loops"] = "3"
        profile = work / "settings.json"
        profile.write_text(json.dumps(settings))
        for name, case in CASES.items():
            model, output = work / (name + ".3mf"), work / (name + ".gcode")
            make_project(model, case)
            run = subprocess.run([binary, "--config", profile, "-o", output, model], cwd=work,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            if run.returncode:
                raise RuntimeError(run.stdout.decode(errors="replace")[-2000:])
            actual[name] = deposition(output)
            if not args.record and actual[name] != expected[name]:
                raise ValueError(f"{name}: expected {expected[name]}, got {actual[name]}")
    if args.record:
        VECTORS.write_text(json.dumps(actual, indent=2) + "\n")
    elif actual.keys() != expected.keys():
        raise ValueError("native vector cases differ")
    print(f"{len(actual)} regional toolpaths verified")


if __name__ == "__main__":
    main()
