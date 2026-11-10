#!/usr/bin/env python3
"""R592: line-level parity by ORDER-RESPECTING alignment (line_compare.py v2).

Why a second instrument. `line_compare.py` (R578/R579) aligns
layer -> feature block -> extrude islands matched by anchor geometry -> windowed
structural walk. That last stage mispairs badly inside a block: on Benchy it
reported 40.39% essentially-identical while an order-INDEPENDENT multiset shows
**75.82% of rust lines are byte-identical to a cpp line** and 99.82% have a
structural counterpart. Its own "aligned" pairs carried relative deviations up to
2.0 (sign flips / near-zero — i.e. unrelated lines paired), and 6,300 blank lines
went unpaired. The 35-point shortfall was the matcher, not the engine.

This version replaces island-anchor matching + windowed walk with a proper
longest-common-subsequence alignment (difflib) over structural keys, run per
(layer, feature) block so the input to each alignment stays small. LCS is
order-respecting, never pairs across a reordering, and degrades gracefully: lines
with no counterpart are reported as inserts/deletes rather than force-matched.

A pair counts as ESSENTIALLY IDENTICAL when the structural keys are equal and
every numeric token agrees to within the tolerance.

Usage:  line_align.py <rust.gcode> <cpp.gcode> [tol]
Both comparators take (rust, bambu) in that order.
"""
import sys
import re
import difflib
from collections import Counter

NUM = re.compile(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?')
TOLS = [0.0, 1e-9, 1e-6, 1e-4, 1e-3, 1e-2]
NAMES = ['exact text', 'rel<=1e-9', 'rel<=1e-6', 'rel<=1e-4', 'rel<=1e-3', 'rel<=1e-2']
SCORE_TOL = 1e-4


def structural(s):
    return NUM.sub('#', s)


def numbers(s):
    return [float(m) for m in NUM.findall(s)]


def reldev(a, b):
    d = abs(a - b)
    m = max(abs(a), abs(b))
    return d if m < 1e-9 else d / m


def split_layers(path):
    """[(layer_idx, [(feature_name, [lines])])] — same segmentation as v1."""
    layers, cur_layer, cur_feat, cur_lines = [], [], None, []

    def flush():
        nonlocal cur_lines
        if cur_lines:
            cur_layer.append((cur_feat, cur_lines))
        cur_lines = []

    with open(path, errors='replace') as fh:
        for raw in fh:
            line = raw.rstrip('\n').rstrip()
            if line.startswith('; CHANGE_LAYER'):
                flush()
                if cur_layer:
                    layers.append(cur_layer)
                cur_layer, cur_feat = [], None
                continue
            if line.startswith('; FEATURE:'):
                flush()
                cur_feat = line.split(':', 1)[1].strip() or '(unnamed)'
                continue
            cur_lines.append(line)
    flush()
    if cur_layer:
        layers.append(cur_layer)
    return layers


def align_block(rlines, clines, acc, per_feat, name, worst):
    """LCS over structural keys; score numerics inside matched runs."""
    rk = [structural(x) for x in rlines]
    ck = [structural(x) for x in clines]
    sm = difflib.SequenceMatcher(None, rk, ck, autojunk=False)
    d = per_feat.setdefault(name, {'aligned': 0, 'ok': 0, 'ronly': 0, 'conly': 0})
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == 'equal':
            for k in range(i2 - i1):
                a, b = numbers(rlines[i1 + k]), numbers(clines[j1 + k])
                acc['aligned'] += 1
                d['aligned'] += 1
                if len(a) != len(b):
                    continue
                dev = max((reldev(x, y) for x, y in zip(a, b)), default=0.0)
                for t, tol in enumerate(TOLS):
                    if dev <= tol:
                        acc['tol'][t] += 1
                if dev <= SCORE_TOL:
                    d['ok'] += 1
                else:
                    acc['worse'] += 1
                    if len(worst) < 4000:
                        worst.append((dev, rlines[i1 + k], clines[j1 + k]))
        else:
            acc['ronly'] += i2 - i1
            acc['conly'] += j2 - j1
            d['ronly'] += i2 - i1
            d['conly'] += j2 - j1


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    rl, cl = split_layers(rp), split_layers(cp)
    print(f"layers: rust {len(rl)}  cpp {len(cl)}")

    acc = {'aligned': 0, 'ronly': 0, 'conly': 0, 'worse': 0, 'tol': [0] * len(TOLS)}
    per_feat, worst, feat_only = {}, [], Counter()

    for li in range(min(len(rl), len(cl))):
        rmap, cmap = {}, {}
        for nm, lines in rl[li]:
            rmap.setdefault(nm or '(pre-feature)', []).append(lines)
        for nm, lines in cl[li]:
            cmap.setdefault(nm or '(pre-feature)', []).append(lines)
        for nm in set(rmap) | set(cmap):
            rb, cb = rmap.get(nm, []), cmap.get(nm, [])
            n = min(len(rb), len(cb))
            for k in range(n):
                align_block(rb[k], cb[k], acc, per_feat, nm, worst)
            for k in range(n, len(rb)):
                acc['ronly'] += len(rb[k])
                feat_only[('rust', nm)] += len(rb[k])
            for k in range(n, len(cb)):
                acc['conly'] += len(cb[k])
                feat_only[('cpp', nm)] += len(cb[k])

    tot_r = acc['aligned'] + acc['ronly']
    tot_c = acc['aligned'] + acc['conly']
    print(f"\nALIGNMENT (layer -> feature block -> LCS over structural keys)")
    print(f"  aligned pairs   {acc['aligned']:>9}")
    print(f"  rust-only lines {acc['ronly']:>9}   ({100*acc['ronly']/max(tot_r,1):5.2f}% of rust body)")
    print(f"  cpp-only lines  {acc['conly']:>9}   ({100*acc['conly']/max(tot_c,1):5.2f}% of cpp body)")

    print(f"\nNUMERIC AGREEMENT among {acc['aligned']} aligned pairs")
    for t, nm in enumerate(NAMES):
        print(f"  {nm:<12} {acc['tol'][t]:>9}  ({100*acc['tol'][t]/max(acc['aligned'],1):6.2f}%)")
    print(f"  {'beyond 1e-2':<12} {acc['worse']:>9}  ({100*acc['worse']/max(acc['aligned'],1):6.2f}%)")

    hl = acc['tol'][3]
    print(f"\n  >>> ESSENTIALLY-IDENTICAL LINES (aligned + rel<=1e-4)")
    print(f"      {100*hl/max(tot_r,1):.2f}% of rust body lines   ({hl}/{tot_r})")
    print(f"      {100*hl/max(tot_c,1):.2f}% of cpp  body lines   ({hl}/{tot_c})")

    print(f"\nPER FEATURE (aligned / essentially-identical / rust-only / cpp-only)")
    for nm in sorted(per_feat, key=lambda n: -per_feat[n]['aligned']):
        d = per_feat[nm]
        tr = d['aligned'] + d['ronly']
        pct = 100 * d['ok'] / max(tr, 1)
        print(f"  {str(nm):<26} {d['aligned']:>8} {d['ok']:>8} ({pct:5.1f}%) {d['ronly']:>7} {d['conly']:>7}")

    if feat_only:
        print(f"\nUNMATCHED FEATURE BLOCKS (present in one engine only)")
        for (eng, nm), n in feat_only.most_common(10):
            print(f"  {eng:<5} {str(nm):<26} {n:>8} lines")

    if worst:
        worst.sort(key=lambda t: -t[0])
        print(f"\nWORST 5 NUMERIC DEVIATIONS (of {acc['worse']} beyond 1e-2)")
        for dev, a, b in worst[:5]:
            print(f"  dev={dev:.4g}\n    R: {a[:100]}\n    C: {b[:100]}")


if __name__ == '__main__':
    main()
