#!/usr/bin/env python3
"""Report rustc unsafe_code diagnostics for the locked production slicer_cli-rust build."""
import argparse
import os
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="Rust target triple (default also checks host build dependencies)")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    command = ["cargo", "check", "--locked", "-p", "slicer-cli-rust", "--bin", "slicer_cli-rust",
               "--message-format=json"]
    if args.target:
        command.extend(["--target", args.target])

    env = os.environ.copy()
    flags = env.get("CARGO_ENCODED_RUSTFLAGS")
    flags = flags.split("\x1f") if flags else env.get("RUSTFLAGS", "").split()
    env["CARGO_ENCODED_RUSTFLAGS"] = "\x1f".join([*flags, "--force-warn", "unsafe_code"])
    counts = Counter()
    # Cargo can suppress registry crates' warnings when replaying cached builds.
    with tempfile.TemporaryDirectory(prefix="slicer-rust-safety-") as build:
        command.extend(["--target-dir", build])
        with subprocess.Popen(command, cwd=root, env=env, stdout=subprocess.PIPE, text=True) as cargo:
            for line in cargo.stdout:
                record = json.loads(line)
                if record["reason"] != "compiler-message":
                    continue
                message = record["message"]
                if (message.get("code") or {}).get("code") == "unsafe_code":
                    counts[record["package_id"]] += 1
                if message["level"] == "error":
                    print(message["rendered"], file=sys.stderr)
            if cargo.wait():
                print("Cargo check failed. Safety results are incomplete.", file=sys.stderr)
                return 2
    for package_id, count in counts.most_common():
        source, _, spec = package_id.rpartition("#")
        label = spec if "@" in spec else f"{source.rsplit('/', 1)[-1]}@{spec}"
        print(f"{count:6}  {label}")
    print(f"{sum(counts.values())} unsafe_code diagnostics in {len(counts)} packages")
    return 1 if counts else 0


if __name__ == "__main__":
    sys.exit(main())
