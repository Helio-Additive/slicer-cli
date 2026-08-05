#!/usr/bin/env python3
"""R578: LINE-LEVEL parity, hierarchically aligned.

"Except for floating point, is every line the same?"

v1 (linecmp.py) failed: it aligned on a structural key alone, but nearly every
extrude line shares the key 'G# X# Y# E#', so once the streams drifted the
two-pointer walk paired unrelated lines and reported sign-flipped nonsense. Its
numbers were an ALIGNMENT ARTEFACT and are not a parity result.

This version aligns hierarchically on markers that are actually rare:

    level 1   '; CHANGE_LAYER'   -> layers        (counts already verified equal)
    level 2   '; FEATURE: <name>' -> feature blocks within a layer
    level 3   windowed structural walk INSIDE one (layer, feature) block

Drift is therefore confined to a single block and cannot propagate. Blocks that
exist in only one engine are reported, never silently dropped.

Usage:  linecmp2.py <rust.gcode> <cpp.gcode> [window]
"""
import sys
import re
from collections import Counter

NUM = re.compile(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?')
TOLS = [0.0, 1e-9, 1e-6, 1e-4, 1e-3, 1e-2]
NAMES = ['exact text', 'rel<=1e-9', 'rel<=1e-6', 'rel<=1e-4', 'rel<=1e-3', 'rel<=1e-2']


def structural(s):
    return NUM.sub('#', s)


def numbers(s):
    return [float(m) for m in NUM.findall(s)]


def reldev(a, b):
    d = abs(a - b)
    m = max(abs(a), abs(b))
    return d if m < 1e-9 else d / m


def split_layers(path):
    """[(layer_idx, [(feature_name, [lines])])]"""
    layers, cur_layer, cur_feat, cur_lines = [], [], None, []

    def flush_feat():
        nonlocal cur_feat, cur_lines
        if cur_lines:
            cur_layer.append((cur_feat, cur_lines))
        cur_feat, cur_lines = cur_feat, []

    with open(path, errors='replace') as fh:
        for raw in fh:
            line = raw.rstrip('\n').rstrip()
            if line.startswith('; CHANGE_LAYER'):
                flush_feat()
                if cur_layer:
                    layers.append(cur_layer)
                cur_layer, cur_feat, cur_lines = [], None, []
                continue
            if line.startswith('; FEATURE:'):
                flush_feat()
                cur_feat = line.split(':', 1)[1].strip() or '(unnamed)'
                continue
            cur_lines.append(line)
    flush_feat()
    if cur_layer:
        layers.append(cur_layer)
    return layers


def islands(lines):
    """Split a feature block into extrude runs separated by travels/other lines.

    Returns [(anchor, lines)] where anchor is the first extruding move's (X,Y),
    used to match runs BETWEEN engines by geometry rather than by emission order
    (R578: the engines order a layer's islands differently, which made an
    order-based walk pair the port loop against the starboard one)."""
    runs, cur, anchor = [], [], None
    for ln in lines:
        is_extrude = (ln[:3] in ('G1 ', 'G2 ', 'G3 ')) and ' E' in ln
        if is_extrude:
            if anchor is None:
                mx = re.search(r'X([-+0-9.]+)', ln)
                my = re.search(r'Y([-+0-9.]+)', ln)
                if mx and my:
                    anchor = (float(mx.group(1)), float(my.group(1)))
            cur.append(ln)
        else:
            if cur:
                runs.append((anchor, cur))
                cur, anchor = [], None
            else:
                cur.append(ln)
                runs.append((None, cur))
                cur = []
    if cur:
        runs.append((anchor, cur))
    return runs


def match_islands(rruns, cruns):
    """Greedy nearest-anchor pairing; unpaired runs are returned separately."""
    used = set()
    pairs, r_un = [], []
    for ra, rl in rruns:
        best, bi = None, None
        for k, (ca, cl) in enumerate(cruns):
            if k in used:
                continue
            if ra is None or ca is None:
                if ra is None and ca is None:
                    best, bi = 0.0, k
                    break
                continue
            d = (ra[0] - ca[0]) ** 2 + (ra[1] - ca[1]) ** 2
            if best is None or d < best:
                best, bi = d, k
        if bi is None:
            r_un.append((ra, rl))
        else:
            used.add(bi)
            pairs.append((rl, cruns[bi][1]))
    c_un = [cruns[k] for k in range(len(cruns)) if k not in used]
    return pairs, r_un, c_un


def walk(rlines, clines, W, acc, worst):
    """Windowed structural walk inside one matched block."""
    rk = [structural(x) for x in rlines]
    ck = [structural(x) for x in clines]
    i = j = 0
    while i < len(rk) and j < len(ck):
        if rk[i] == ck[j]:
            a, b = numbers(rlines[i]), numbers(clines[j])
            if len(a) == len(b):
                dev = max((reldev(x, y) for x, y in zip(a, b)), default=0.0)
                for t, tol in enumerate(TOLS):
                    if dev <= tol:
                        acc['tol'][t] += 1
                if dev > TOLS[-1]:
                    acc['worse'] += 1
                    if len(worst) < 5000:
                        worst.append((dev, rlines[i], clines[j]))
            acc['aligned'] += 1
            i += 1
            j += 1
            continue
        best = None
        for d in range(1, W):
            if i + d < len(rk) and rk[i + d] == ck[j]:
                best = (d, 0)
                break
            if j + d < len(ck) and rk[i] == ck[j + d]:
                best = (0, d)
                break
        if best is None:
            acc['ronly'] += 1
            acc['conly'] += 1
            i += 1
            j += 1
        else:
            di, dj = best
            acc['ronly'] += di
            acc['conly'] += dj
            i += di
            j += dj
    acc['ronly'] += len(rk) - i
    acc['conly'] += len(ck) - j


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    W = int(sys.argv[3]) if len(sys.argv) > 3 else 200

    rl, cl = split_layers(rp), split_layers(cp)
    print(f"layers: rust {len(rl)}  cpp {len(cl)}")

    acc = {'aligned': 0, 'ronly': 0, 'conly': 0, 'worse': 0, 'tol': [0] * len(TOLS)}
    worst = []
    feat_only = Counter()
    per_feat = {}
    nlayers = min(len(rl), len(cl))

    for li in range(nlayers):
        rfeats, cfeats = rl[li], cl[li]
        rmap, cmap = {}, {}
        for name, lines in rfeats:
            rmap.setdefault(name or '(pre-feature)', []).append(lines)
        for name, lines in cfeats:
            cmap.setdefault(name or '(pre-feature)', []).append(lines)
        for name in set(rmap) | set(cmap):
            rblocks, cblocks = rmap.get(name, []), cmap.get(name, [])
            n = min(len(rblocks), len(cblocks))
            for k in range(n):
                before = dict(acc)
                before_tol = list(acc['tol'])
                pairs, r_un, c_un = match_islands(islands(rblocks[k]), islands(cblocks[k]))
                for rl_, cl_ in pairs:
                    walk(rl_, cl_, W, acc, worst)
                for _, rl_ in r_un:
                    acc['ronly'] += len(rl_)
                for _, cl_ in c_un:
                    acc['conly'] += len(cl_)
                d = per_feat.setdefault(name, {'aligned': 0, 'ok': 0, 'ronly': 0, 'conly': 0})
                d['aligned'] += acc['aligned'] - before['aligned']
                d['ok'] += acc['tol'][3] - before_tol[3]
                d['ronly'] += acc['ronly'] - before['ronly']
                d['conly'] += acc['conly'] - before['conly']
            for k in range(n, len(rblocks)):
                feat_only[('rust', name)] += len(rblocks[k])
                acc['ronly'] += len(rblocks[k])
            for k in range(n, len(cblocks)):
                feat_only[('cpp', name)] += len(cblocks[k])
                acc['conly'] += len(cblocks[k])

    tot_r = acc['aligned'] + acc['ronly']
    tot_c = acc['aligned'] + acc['conly']
    print(f"\nALIGNMENT (layer -> feature block -> windowed structural walk)")
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
    for name in sorted(per_feat, key=lambda n: -per_feat[n]['aligned']):
        d = per_feat[name]
        pct = 100 * d['ok'] / max(d['aligned'], 1)
        print(f"  {name:<26} {d['aligned']:>8} {d['ok']:>8} ({pct:5.1f}%) {d['ronly']:>7} {d['conly']:>7}")

    if feat_only:
        print(f"\nUNMATCHED FEATURE BLOCKS (whole blocks present in one engine only)")
        for (eng, name), n in feat_only.most_common(12):
            print(f"  {eng:<5} {name:<26} {n:>8} lines")

    if worst:
        worst.sort(key=lambda t: -t[0])
        print(f"\nWORST 6 NUMERIC DEVIATIONS (of {acc['worse']} beyond 1e-2)")
        for dev, a, b in worst[:6]:
            print(f"  dev={dev:.4g}\n    R: {a[:105]}\n    C: {b[:105]}")


if __name__ == '__main__':
    main()
