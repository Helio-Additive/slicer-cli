#!/usr/bin/env bash
# spike/layout/repeat.sh — DISPOSABLE fixed-seed repeatability harness (issue #7 spike).
# usage: repeat.sh <path-to-layout_spike>
# Runs each probe fixture 10x with --seed 42 and checks output_sha256 stability,
# then compares --parallel 0 vs --parallel 1 on five_mixed_sizes.
set -u

BIN="${1:?usage: repeat.sh <path-to-layout_spike>}"
DIR="$(cd "$(dirname "$0")" && pwd)"
FIX="$DIR/fixtures"
RUNS=10

# libslic3r static init emits one boost-log trace line to stdout before main();
# parse JSON from the first '{'.
PARSE="import json,sys; s=sys.stdin.read(); print(json.loads(s[s.index('{'):])['output_sha256'])"

FIXTURES="two_rectangles five_mixed_sizes l_shaped_bed locked_center_item exclusion_zone rotations_45_90"

printf '%-22s %-66s %s\n' "fixture" "output_sha256 (first run)" "consistent (${RUNS}x)"
overall=0
for name in $FIXTURES; do
  first=""
  ok="yes"
  for i in $(seq 1 $RUNS); do
    h=$("$BIN" "$FIX/$name.json" --seed 42 2>/dev/null | python3 -c "$PARSE")
    if [ -z "$first" ]; then first="$h"; fi
    if [ "$h" != "$first" ]; then ok="no"; overall=1; fi
  done
  printf '%-22s %-66s %s\n' "$name" "$first" "$ok"
done

# parallel on/off comparison
hp0=$("$BIN" "$FIX/five_mixed_sizes.json" --seed 42 --parallel 0 2>/dev/null | python3 -c "$PARSE")
hp1=$("$BIN" "$FIX/five_mixed_sizes.json" --seed 42 --parallel 1 2>/dev/null | python3 -c "$PARSE")
if [ "$hp0" = "$hp1" ]; then
  echo "parallel 0 vs 1 (five_mixed_sizes): MATCH ($hp0)"
else
  echo "parallel 0 vs 1 (five_mixed_sizes): DIFFER"
  echo "  parallel=0: $hp0"
  echo "  parallel=1: $hp1"
fi
exit $overall
