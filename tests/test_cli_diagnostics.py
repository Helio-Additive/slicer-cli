"""Check input refusals and their structured diagnostics with the packaged CLI."""
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile


def events(stdout):
    prefix = "[[SLICER_EVENT]] "
    return [json.loads(line[len(prefix):]) for line in stdout.splitlines() if line.startswith(prefix)]


def argument_errors(binary, engine, work):
    # Early startup errors go to stderr. Slice-mode errors also go to result.json.
    cases = [
        (["--calib-start", "bad"], "--calib-start expects a number, got 'bad'", False),
        (["--calib-extruder-id", "bad"], "--calib-extruder-id expects an integer, got 'bad'", False),
        (["--export-3mf", "out.3mf"], "--export-3mf needs --slice", False),
        (["--arrange", "1"], "--arrange needs --slice (the official CLI arranges inside its plate loop)", False),
        (["--slice", "1", "--calib-step", "x"], "--calib-step expects a number, got 'x'", True),
    ]
    assemble_hint = ("give no model files with it." if engine == "bambu"
                     else "give no model files and no transforms with it.")
    for args, message in (
        (["--debug", "2"], "No input file specified"),
        (["--plate", "1"], "--plate and --slice are mutually exclusive; use --slice N for plate N"),
        (["--engine-info", "model.stl"], "--engine-info describes a file and slices nothing; give it without --slice."),
        (["--list-presets"], "--list-presets lists this engine's presets and slices nothing; give it without --slice."),
        (["--layout-plan"], "--layout-plan arranges the input's objects and slices nothing; give it without --slice."),
        (["--load-assemble-list", "plates.json", "model.stl"],
         "--load-assemble-list builds the plates from its list; " + assemble_hint),
    ):
        cases.append((["--slice", "1", *args], message, True))
    app = {"bambu": "BambuStudio", "orca": "OrcaSlicer", "snapmaker": "Snapmaker Orca"}[engine]
    for flag, hint in (
        ("--wall-loops", "a whole number"),
        ("--layer-height", "a number"),
        ("--sparse-infill-density", "a percentage such as 15%"),
        ("--line-width", "a number" if engine == "bambu" else "a number or a percentage"),
        ("--enable-support", "1 or 0"),
    ):
        message = f"{flag}: 'bad' is not a value {app} takes for this flag; it takes {hint}"
        cases.append((["--slice", "1", flag + "=bad"], message, True))
    cases.append((["--slice", "1", "--metadata-name", "sample"],
                  "--metadata-name and --metadata-value must come in pairs", True))
    if engine == "snapmaker":
        cases.append((["--slice", "1", "--allow-mix-temp=2"], "--allow-mix-temp takes =0 or =1, not '2'", True))
    for index, (args, message, slice_mode) in enumerate(cases):
        output = work / str(index)
        output.mkdir()
        run = subprocess.run([binary, *args], cwd=output, capture_output=True, encoding="utf-8")
        assert run.returncode != 0, args
        assert "Error: " + message in run.stderr.splitlines(), (args, run.stderr)
        result_file = output / "result.json"
        if slice_mode:
            result = json.loads(result_file.read_text(encoding="utf-8"))
            assert result["return_code"] == -2, result
            assert result["error_string"] == "Invalid parameters to the slicer. " + message, result
        else:
            assert run.returncode == 1, args
            assert not result_file.exists(), args


def rejected_overrides(binary, fixture, work):
    output = work / "overrides"
    output.mkdir()
    run = subprocess.run(
        [binary, fixture, "--slice", "1", "--nozzle", "invalid", "--bed-temp", "invalid",
         "--export-3mf", "res.3mf", "--outputdir", output],
        cwd=output, capture_output=True, encoding="utf-8",
    )
    assert run.returncode == 0, run.stderr
    rejected = [e for e in events(run.stdout) if e.get("event") == "override_rejected"]
    # Export applies the overrides again. Each rejected value is reported once.
    assert [e["opt_key"] for e in rejected] == ["bed_temperature", "nozzle_diameter"], rejected
    for event in rejected:
        assert event["tag"] == "InvalidOverrideValue", event
        assert event["value"] == "invalid", event
        assert event["message"] == f"Command-line override for '{event['opt_key']}' was rejected and had no effect", event
    assert (output / "plate_1.gcode").stat().st_size > 0
    assert zipfile.is_zipfile(output / "res.3mf")


def file_info_errors(binary, work):
    for filename, message in (
        ("missing.stl", "The input files to the slicer are not found."),
        ("empty.stl", "The input model file to the slicer can not be parsed. empty.stl is not a readable STL file."),
        ("empty.obj", "The input model file to the slicer can not be parsed. empty.obj is not a readable obj file."),
        ("empty.3mf", "The input model file to the slicer can not be parsed. empty.3mf is not a readable 3MF archive with a 3D/3dmodel.model part."),
    ):
        if filename.startswith("empty"):
            (work / filename).touch()
        run = subprocess.run([binary, "--engine-info", filename], cwd=work, capture_output=True, encoding="utf-8")
        info = json.loads(run.stdout)
        assert run.returncode != 0, info
        assert info["file"] == filename and info["error"] == message, info


def layout_input_errors(binary, work):
    path = work / "layout.json"
    for data, message in (
        ({}, "objects must be a JSON array"),
        ({"objects": [1]}, "each object entry must be a JSON object"),
        ({"profiles": {}}, "profilesDir is required when profiles is specified"),
    ):
        path.write_text(json.dumps(data), encoding="utf-8")
        run = subprocess.run([binary, "--layout", path], cwd=work, capture_output=True, encoding="utf-8")
        assert run.returncode == 1 and message in run.stderr.splitlines(), run.stderr
        assert not events(run.stdout), run.stdout

    missing = subprocess.run([binary, "--layout-plan", "--input", "missing.json"],
                             cwd=work, capture_output=True, encoding="utf-8")
    assert missing.returncode == 3, missing.stderr
    assert json.loads(missing.stderr.splitlines()[-1]) == {
        "schemaVersion": 1, "error": {"code": "INVALID_INPUT", "message": "cannot open --input file"}
    }
    invalid = subprocess.run([binary, "--layout-plan"], input="{", cwd=work, capture_output=True, encoding="utf-8")
    error = json.loads(invalid.stderr.splitlines()[-1])
    assert invalid.returncode == 3 and error["schemaVersion"] == 1, error
    assert error["error"]["code"] == "INVALID_INPUT", error
    assert error["error"]["message"].startswith("JSON parse error: "), error


def snapmaker_newer_project(binary, fixture, work):
    future = work / "future.3mf"
    with zipfile.ZipFile(fixture) as source, zipfile.ZipFile(future, "w") as target:
        for member in source.infolist():
            data = source.read(member)
            if member.filename == "3D/3dmodel.model":
                data, count = re.subn(
                    rb'(<metadata name="Application">Snapmaker_Orca-)[^<]+',
                    rb'\g<1>99.1.0',
                    data,
                )
                assert count == 1
            target.writestr(member, data)

    warning = "Forcing CLI newer-file compatibility from SNAPMAKER_ORCA_ALLOW_NEWER_FILE"
    for setting, expected in (("0", -24), ("1", 0)):
        output = work / ("newer-" + setting)
        run = subprocess.run(
            [binary, future, "--slice", "1", "--outputdir", output],
            env=dict(os.environ, SNAPMAKER_ORCA_ALLOW_NEWER_FILE=setting),
            cwd=work, capture_output=True, encoding="utf-8",
        )
        result = json.loads((output / "result.json").read_text(encoding="utf-8"))
        assert result["return_code"] == expected, result
        assert (run.returncode == 0) == (expected == 0), run.stderr
        assert (warning in run.stderr.splitlines()) == (setting == "1"), run.stderr
        gated = any(e.get("tag") == "FileVersionNewerThanEngine" for e in events(run.stdout))
        assert gated == (setting == "0"), run.stdout
        assert (output / "plate_1.gcode").is_file() == (setting == "1")


if __name__ == "__main__":
    binary = Path(shutil.which(sys.argv[1]) or sys.argv[1]).resolve()
    engine = sys.argv[2]
    fixture = Path(sys.argv[3]).resolve()
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        argument_errors(binary, engine, work)
        file_info_errors(binary, work)
        layout_input_errors(binary, work)
        rejected_overrides(binary, fixture, work)
        if engine == "snapmaker":
            snapmaker_newer_project(binary, fixture, work)
