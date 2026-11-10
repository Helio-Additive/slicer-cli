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


def dump_divergence(rg, cg, dec, feat_want, n, ctx):
    """R653: for the n worst groups of one feature, show where the sequences part.

    The matching blocks already know the answer — the first index not covered by
    a block is the first divergence. Printing that neighbourhood raw is the only
    way to tell a systematic per-loop emission-order difference from scatter.
    """
    scored = []
    for key in set(rg) | set(cg):
        layer, feat = key
        if feat != feat_want:
            continue
        r = [quantise(x, dec) for x in rg.get(key, [])]
        c = [quantise(x, dec) for x in cg.get(key, [])]
        if not r or not c:
            continue
        blocks = SequenceMatcher(None, r, c, autojunk=False).get_matching_blocks()
        o = sum(b.size for b in blocks)
        scored.append((len(r) - o, layer, r, c, blocks, rg.get(key, []), cg.get(key, [])))
    scored.sort(reverse=True, key=lambda t: t[0])
    print(f"feature {feat_want!r}: {len(scored)} groups; showing the {min(n, len(scored))} worst\n")
    for loss, layer, r, c, blocks, rraw, craw in scored[:n]:
        # first index of ours not covered by a matching block
        i = 0
        for b in blocks:
            if b.a > i:
                break
            i = b.a + b.size
        j = 0
        for b in blocks:
            if b.a + b.size > i:
                j = b.b
                break
            j = b.b + b.size
        print(f"=== layer {layer}  ours {len(r)} lines, cpp {len(c)}, out-of-order {loss}")
        print(f"    first divergence at ours[{i}] / cpp[{j}]")
        lo_r, hi_r = max(0, i - ctx), min(len(rraw), i + ctx)
        lo_c, hi_c = max(0, j - ctx), min(len(craw), j + ctx)
        width = max(hi_r - lo_r, hi_c - lo_c)
        for k in range(width):
            ri, ci = lo_r + k, lo_c + k
            rl = rraw[ri][:52] if ri < hi_r else ""
            cl = craw[ci][:52] if ci < hi_c else ""
            mark = ">>" if (ri == i or ci == j) else "  "
            print(f"  {mark} {ri:>5} {rl:<52} | {ci:>5} {cl}")
        print()


def main():
    argv = list(sys.argv[1:])
    feat_want = None
    n_dump, ctx = 3, 12
    if '--dump-divergence' in argv:
        k = argv.index('--dump-divergence')
        feat_want = argv[k + 1]
        rest = argv[k + 2:]
        n_dump = int(rest[0]) if rest and rest[0].isdigit() else n_dump
        argv = argv[:k]

    rp, cp = argv[0], argv[1]
    dec = int(argv[2]) if len(argv) > 2 else DEC
    rg, cg = groups(rp), groups(cp)

    if feat_want is not None:
        dump_divergence(rg, cg, dec, feat_want, n_dump, ctx)
        return

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
