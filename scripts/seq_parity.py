#!/usr/bin/env python3
"""R652: line-level parity that RESPECTS ORDER — the companion to `line_parity.py`.

Why a fourth instrument.

  `line_parity.py` (v3) matches by tolerant MULTISET intersection inside each
  (layer, feature) group. Its own docstring calls that an upper bound, but the
  practical consequence went unnoticed for fifty rounds: **it cannot see intra-group
  ordering at all.** R651 moved 15,081 `M204` lines from one side of `; WIPE_START`
  to the other — into the position C++ puts them — and v3's matched count did not
  move by one. A file could score highly with every feature's internals shuffled.

  `line_align.py` (v2) does respect order, but aligns on STRUCTURAL KEYS
  (`G1 X# Y# E#`), which are degenerate: inside a block every alignment scores
  equally and difflib picks one arbitrarily, so its "aligned" pairs mix true
  matches with coin flips.

This instrument takes v3's grouping and v3's quantisation — so a "match" is still
"the same line to 1e-3 mm" and never an arbitrary pairing — and replaces the
multiset intersection with a longest-matching-block walk over the two sequences.
A line counts only if it is essentially identical AND reachable in order.

  in_order <= matched  always. The gap between them IS the ordering defect,
  measured for the first time.

difflib's `get_matching_blocks` is a recursive longest-block heuristic, not a
strict LCS, so `in_order` is a lower bound on the true in-order count. It never
over-reports, which is the direction that matters here.

Usage:  seq_parity.py <rust.gcode> <cpp.gcode> [decimals]
Takes (rust, bambu) in that order, like the other comparators.
"""
import sys
from collections import Counter, defaultdict
from difflib import SequenceMatcher

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from line_parity import DEC, groups, quantise  # noqa: E402

# A pair of groups larger than this would make the quadratic fallback in difflib
# uncomfortable. Nothing in Benchy or Majora comes close (largest observed group
# is 7,360 lines), but a silent cap would be a lie about coverage, so anything
# skipped is counted and printed.
MAX_GROUP = 60000


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    dec = int(sys.argv[3]) if len(sys.argv) > 3 else DEC
    rg, cg = groups(rp), groups(cp)

    tot_r = sum(len(v) for v in rg.values())
    tot_c = sum(len(v) for v in cg.values())
    matched = 0        # v3's order-blind multiset count, recomputed here
    in_order = 0       # order-respecting count
    skipped = 0
    per_feat = defaultdict(lambda: {'r': 0, 'c': 0, 'm': 0, 'o': 0})

    for key in set(rg) | set(cg):
        _, feat = key
        r = [quantise(x, dec) for x in rg.get(key, [])]
        c = [quantise(x, dec) for x in cg.get(key, [])]

        m = sum((Counter(r) & Counter(c)).values())

        if max(len(r), len(c)) > MAX_GROUP:
            skipped += len(r)
            o = 0
        else:
            o = sum(b.size for b in
                    SequenceMatcher(None, r, c, autojunk=False).get_matching_blocks())

        matched += m
        in_order += o
        d = per_feat[feat]
        d['r'] += len(r)
        d['c'] += len(c)
        d['m'] += m
        d['o'] += o

    print(f"tolerance: numbers rounded to {dec} decimals (1e-{dec} mm on coordinates)")
    print(f"grouping : (layer, feature); within a group, order-respecting match\n")
    print(f"body lines: rust {tot_r}  cpp {tot_c}")
    if skipped:
        print(f"SKIPPED {skipped} rust lines in groups over {MAX_GROUP} — NOT counted as in-order")
    print(f"\n  >>> ESSENTIALLY-IDENTICAL LINES")
    print(f"      content  {100*matched/max(tot_r,1):.2f}% of rust body   ({matched}/{tot_r})")
    print(f"      IN ORDER {100*in_order/max(tot_r,1):.2f}% of rust body   ({in_order}/{tot_r})")
    print(f"      ordering loss: {matched-in_order} lines "
          f"({100*(matched-in_order)/max(matched,1):.2f}% of content matches)")

    print(f"\nPER FEATURE (rust / content-matched / in-order / in-order % of rust)")
    for feat in sorted(per_feat, key=lambda f: -per_feat[f]['r']):
        d = per_feat[feat]
        print(f"  {str(feat):<26} {d['r']:>8} {d['m']:>8} {d['o']:>8} "
              f"({100*d['o']/max(d['r'],1):5.1f}%)")


if __name__ == '__main__':
    main()
