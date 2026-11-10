#!/usr/bin/env python3
"""R593: line-level parity by TOLERANT MULTISET MATCH — the alignment-free measure.

Why a third instrument.

  v1 `line_compare.py`  island anchors + windowed walk. Mispaired badly; scored
                        Benchy 40.39% when 75.82% of lines are byte-identical.
  v2 `line_align.py`    difflib LCS over structural keys. Better (52.51%) and
                        order-respecting, but STILL partly measures itself: the
                        structural key of nearly every extrude line is the same
                        (`G1 X# Y# E#`), so inside a block every alignment scores
                        equally and difflib picks one arbitrarily. Its "aligned"
                        pairs carry deviations up to 51 mm on a 60 mm model while
                        75% of them are exact — a bimodal mix of true matches and
                        arbitrary ones.

Both try to decide WHICH line corresponds to which. That question has no unique
answer when the keys are degenerate, and it is not the question being asked. The
question is: **for each line we emit, does the other engine emit essentially the
same line?** That is a multiset containment test, and it needs no alignment.

Method: within each (layer, feature) group — so a line only matches a line from
the same place in the print — quantise every numeric token to the tolerance and
count multiset intersection. Order-independent, immune to alignment ambiguity,
and symmetric. Reported both ways since the files differ slightly in length.

Because it ignores order, it is an UPPER bound on line-for-line identity in the
same way the aligners are lower bounds; quote it together with the v2 figure and
with the body line-count gap. Both together bracket the truth.

Usage:  line_parity.py <rust.gcode> <cpp.gcode> [decimals]
Takes (rust, bambu) in that order, like the other comparators.
"""
import sys
import re
from collections import Counter, defaultdict

NUM = re.compile(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?')
DEC = 3          # 1e-3 mm = 1 micron on coordinates


def quantise(s, dec):
    """Line text with every numeric token rounded to `dec` decimals."""
    def q(m):
        return f"{round(float(m.group(0)), dec):.{dec}f}"
    return NUM.sub(q, s)


def groups(path):
    """{(layer_idx, feature): [lines]} — same segmentation as the aligners."""
    out = defaultdict(list)
    layer, feat = 0, '(pre-feature)'
    with open(path, errors='replace') as fh:
        for raw in fh:
            line = raw.rstrip('\n').rstrip()
            if line.startswith('; CHANGE_LAYER'):
                layer += 1
                feat = '(pre-feature)'
                continue
            if line.startswith('; FEATURE:'):
                feat = line.split(':', 1)[1].strip() or '(unnamed)'
                continue
            if line:
                out[(layer, feat)].append(line)
    return out


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    dec = int(sys.argv[3]) if len(sys.argv) > 3 else DEC
    rg, cg = groups(rp), groups(cp)

    tot_r = sum(len(v) for v in rg.values())
    tot_c = sum(len(v) for v in cg.values())
    matched = 0
    per_feat = defaultdict(lambda: {'r': 0, 'c': 0, 'm': 0})

    for key in set(rg) | set(cg):
        _, feat = key
        r, c = rg.get(key, []), cg.get(key, [])
        cr = Counter(quantise(x, dec) for x in r)
        cc = Counter(quantise(x, dec) for x in c)
        m = sum((cr & cc).values())
        matched += m
        d = per_feat[feat]
        d['r'] += len(r)
        d['c'] += len(c)
        d['m'] += m

    print(f"tolerance: numbers rounded to {dec} decimals (1e-{dec} mm on coordinates)")
    print(f"grouping : (layer, feature) — a line can only match one from the same block\n")
    print(f"body lines: rust {tot_r}  cpp {tot_c}   line-count gap "
          f"{abs(tot_r-tot_c)/max(tot_r,tot_c):.2%}")
    print(f"\n  >>> ESSENTIALLY-IDENTICAL LINES (tolerant multiset, order-independent)")
    print(f"      {100*matched/max(tot_r,1):.2f}% of rust body lines   ({matched}/{tot_r})")
    print(f"      {100*matched/max(tot_c,1):.2f}% of cpp  body lines   ({matched}/{tot_c})")

    print(f"\nPER FEATURE (rust lines / matched / % of rust / cpp lines)")
    for feat in sorted(per_feat, key=lambda f: -per_feat[f]['r']):
        d = per_feat[feat]
        print(f"  {str(feat):<26} {d['r']:>8} {d['m']:>8} ({100*d['m']/max(d['r'],1):5.1f}%) {d['c']:>8}")


if __name__ == '__main__':
    main()
