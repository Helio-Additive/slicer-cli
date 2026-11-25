#!/usr/bin/env python3
"""Every setting an official command line handles is taken by slicer-cli.

The official programs read some settings from the flags themselves
(m_extra_config in BambuStudio.cpp, OrcaSlicer.cpp, Snapmaker_Orca.cpp) and do
more with them than lay them over the loaded settings. This test lists those
settings from the pinned source and fails when the matching slicer-cli binary
refuses one as a flag (--list-settings, refused_as_flag) that
tests/official-cli-refused-settings.json does not list with its reason. It
fails on a listed setting that is no longer refused too, so the list stays
true. An engine update that brings a new official flag setting therefore fails
here until the setting is ported.

Usage: tests/test_official_cli_parity.py BINARY... [--references DIR]
"""
import json
import os
import re
import subprocess
import sys

SOURCES = {
    "bambustudio": "BambuStudio/src/BambuStudio.cpp",
    "orcaslicer": "OrcaSlicer/src/OrcaSlicer.cpp",
    "snapmaker": "SnapmakerOrca/src/Snapmaker_Orca.cpp",
}
# m_extra_config.has("key"), .option<Type<Nested>>("key"), .opt_bool("key"), ...
READ = re.compile(r'm_extra_config\s*\.\s*\w+\s*(?:<[^()"]*>)?\s*\(\s*"([a-z0-9_]+)"')


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    args = sys.argv[1:]
    references = os.path.join(here, "..", "references")
    if "--references" in args:
        at = args.index("--references")
        references = args[at + 1]
        del args[at:at + 2]
    if not args:
        sys.exit(__doc__)
    allowed = json.load(open(os.path.join(here, "official-cli-refused-settings.json"), encoding="utf-8"))
    failed = False
    for binary in args:
        listing = json.loads(subprocess.run([binary, "--list-settings"], check=True, capture_output=True).stdout)
        engine = listing["engine"]
        source = os.path.join(references, SOURCES[engine])
        official = set(READ.findall(open(source, encoding="utf-8", errors="replace").read()))
        if not official:
            print("FAIL: [%s] no setting read from the flags found in %s" % (engine, source))
            failed = True
            continue
        refused = {s["key"] for s in listing["settings"] if "refused_as_flag" in s}
        listed = set(allowed.get(engine, {}))
        unlisted = sorted((official & refused) - listed)
        stale = sorted(listed - (official & refused))
        for key in unlisted:
            print("FAIL: [%s] the official command line handles %s and this binary refuses it as a flag; port it, or "
                  "list it in tests/official-cli-refused-settings.json with the owner's agreement" % (engine, key))
        for key in stale:
            print("FAIL: [%s] tests/official-cli-refused-settings.json lists %s, which this binary does not refuse "
                  "or the official command line no longer handles; take it out" % (engine, key))
        failed = failed or bool(unlisted or stale)
        if not unlisted and not stale:
            print("PASS: [%s] %d settings the official command line handles (%s), %d refused with a listed reason"
                  % (engine, len(official), ", ".join(sorted(official)), len(official & refused)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
