#!/usr/bin/env python3
"""Lines of the oracle's G-code body that the other file has verbatim.

    exact_lines.py <rust.gcode> <oracle.gcode> [n]

`seq_parity.py` scores moves in order and leaves comments, blank lines and most
M-codes out. Byte-identical output needs those too, so this counts every line
of the body (from the first `; CHANGE_LAYER` to `; EXECUTABLE_BLOCK_END`) as a
multiset: how many of the oracle's lines the other file also has, exactly as
written. With `n` it lists the n line shapes (numbers replaced by `#`, moves
left out) that are missing most often on either side.
"""
import collections
import re
import sys


def body(path):
    lines = collections.Counter()
    inside = False
    with open(path, errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("; CHANGE_LAYER"):
                inside = True
            if line.startswith("; EXECUTABLE_BLOCK_END"):
                inside = False
            if inside:
                lines[line] += 1
    return lines


def shape(line):
    if line[:2] in ("G0", "G1", "G2", "G3"):
        return None
    return re.sub(r"-?\d+\.?\d*", "#", line)[:60]


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    rust, oracle = body(sys.argv[1]), body(sys.argv[2])
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    total, common = sum(oracle.values()), sum((oracle & rust).values())
    print("oracle body lines %d, other %d, verbatim in both %d (%.2f%% of the oracle's)"
          % (total, sum(rust.values()), common, 100.0 * common / max(1, total)))
    if not n:
        return
    only_oracle, only_rust = collections.Counter(), collections.Counter()
    for line, k in (oracle - rust).items():
        s = shape(line)
        if s is not None:
            only_oracle[s] += k
    for line, k in (rust - oracle).items():
        s = shape(line)
        if s is not None:
            only_rust[s] += k
    print("%8s %8s  shape" % ("oracle", "other"))
    for s in sorted(set(only_oracle) | set(only_rust), key=lambda s: -(only_oracle[s] + only_rust[s]))[:n]:
        print("%8d %8d  %s" % (only_oracle[s], only_rust[s], s if s else "(empty line)"))


if __name__ == "__main__":
    main()
