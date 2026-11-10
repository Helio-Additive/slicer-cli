#!/usr/bin/env python3
"""R708: WHAT KIND of line is out of order?

`seq_parity.py` reports the ordering loss as a single number. That number is
actionable only once you know what is IN it: extrusion geometry being visited
in a different sequence is a chaining/seam problem, while per-path header lines
(`; LINE_WIDTH:`, `M204`, a speed-only `G1 F`) being out of order is an
emission-order problem in one function -- completely different fixes.

Method. For each (layer, feature) group, align with the same matcher
`seq_parity` uses, then identify exactly the lines it counts as
"content-matched but not in order":

    per distinct line text,
        leftover = min(count_rust, count_cpp) - aligned_count

and assign those leftovers to the uncovered occurrences in order. Those are the
lines the ordering loss is made of. Then classify them.

RESULT (R708, benchy classic, 17,702 lost lines):

    G1 extrude XY+E            6603  37.3%   <- real path geometry
    ; LINE_WIDTH:              2816  15.9%   -+
    M204                       2068  11.7%    |  per-path header lines:
    G1 speed-only F            1933  10.9%    |  8,138 lines = 46%
    ; COOLING_NODE:             749   4.2%    |
    ; WIPE_START / ; WIPE_END   572   3.2%   -+
    G2 / G3 arcs                802   4.5%
    travel / retract           1716   9.7%

  46% of classic's ordering loss is the header block around each path, which is
  what sent R708 to the `; LINE_WIDTH:`-before-speed inversion (the parked R654
  gate). The remaining 37% is genuine path-visit order.

Usage: order_loss_kinds.py <rust.gcode> <cpp.gcode> [feature]
"""
import re
import sys
from collections import Counter, defaultdict
from difflib import SequenceMatcher

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from line_parity import DEC, groups, quantise  # noqa: E402

# Groups larger than this are skipped rather than run through difflib's
# quadratic fallback; the skipped total is printed so coverage is never silent.
MAX_GROUP = 20000


def kind(line):
    """A label coarse enough to aggregate, fine enough to act on."""
    s = line.strip()
    if s.startswith(';'):
        return 'comment ' + ' '.join(s.split()[:2])[:30]
    if s.startswith('G1') or s.startswith('G0'):
        has_e = re.search(r'\bE-?[\d.]', s) is not None
        has_xy = re.search(r'\b[XY]-?[\d.]', s) is not None
        has_z = re.search(r'\bZ-?[\d.]', s) is not None
        has_f = re.search(r'\bF[\d.]', s) is not None
        if has_e and has_xy:
            return 'G1 extrude XY+E'
        if has_e:
            return 'G1 retract/prime E-only'
        if has_z:
            return 'G1 travel Z'
        if has_xy:
            return 'G1 travel XY' + (' +F' if has_f else '')
        return 'G1 speed-only F' if has_f else 'G1 other'
    return s.split()[0][:24] if s.split() else '(blank)'


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    want = sys.argv[3] if len(sys.argv) > 3 else None
    rg, cg = groups(rp), groups(cp)

    kinds = Counter()
    kinds_by_feat = defaultdict(Counter)
    total = skipped = 0

    for key in sorted(set(rg) | set(cg)):
        _, feat = key
        if want and feat != want:
            continue
        rraw, craw = rg.get(key, []), cg.get(key, [])
        r = [quantise(x, DEC) for x in rraw]
        c = [quantise(x, DEC) for x in craw]
        if not r or not c:
            continue
        if max(len(r), len(c)) > MAX_GROUP:
            skipped += len(r)
            continue

        blocks = SequenceMatcher(None, r, c, autojunk=False).get_matching_blocks()
        cov_a = set()
        aligned = Counter()
        for b in blocks:
            for k in range(b.size):
                cov_a.add(b.a + k)
                aligned[r[b.a + k]] += 1

        cr, cc = Counter(r), Counter(c)
        leftover = Counter()
        for line in cr:
            n = min(cr[line], cc.get(line, 0)) - aligned.get(line, 0)
            if n > 0:
                leftover[line] = n
        if not leftover:
            continue

        for i in range(len(r)):
            if i in cov_a or leftover.get(r[i], 0) <= 0:
                continue
            leftover[r[i]] -= 1
            k = kind(rraw[i])
            kinds[k] += 1
            kinds_by_feat[feat][k] += 1
            total += 1

    scope = f"feature {want!r}" if want else "ALL features"
    print(f"rust={rp}\ncpp ={cp}\nscope: {scope}")
    if skipped:
        print(f"SKIPPED {skipped} rust lines in groups over {MAX_GROUP}")
    print(f"\nOUT-OF-ORDER LINES BY KIND  (total {total})")
    for k, n in kinds.most_common(18):
        print(f"  {k:<32} {n:>8}  {100*n/max(total,1):5.1f}%")

    if not want:
        print(f"\nTOP FEATURES x TOP KINDS")
        for feat, cnt in sorted(kinds_by_feat.items(),
                                key=lambda kv: -sum(kv[1].values()))[:6]:
            tot = sum(cnt.values())
            top = ', '.join(f"{k}={n}" for k, n in cnt.most_common(4))
            print(f"  {feat:<26} {tot:>7}   {top}")


if __name__ == '__main__':
    main()
