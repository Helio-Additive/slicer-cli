#!/usr/bin/env python3
"""R709: is the path-geometry ordering loss ISLAND SET, ISLAND ORDER, or
WITHIN-ISLAND (seam)? And what is the divergence WORTH?

`order_loss_kinds.py` (R708) showed that 37% of benchy classic's ordering loss
is real path geometry rather than per-path header lines. That is still three
different defects with three different fixes, and the divergence dumps cannot
separate them by eye -- especially since we emit G2/G3 arcs where C++ emits G1
segments, which defeats any point-level comparison.

Method. Split each (layer, feature) group into ISLANDS: a maximal run of
extrusion moves, broken by any travel (an XY move with no positive E). Key each
island by its BOUNDING BOX rounded to `res` mm. Arc fitting preserves the traced
shape, so the bbox survives the G1-vs-G2/G3 difference, and a bbox is invariant
to where the loop is seamed -- so a key match means "the same island", however
it was drawn.

Then, per layer, three questions in order:
  1. same NUMBER of islands?
  2. same SET of island keys?      no -> the geometry differs, not the ordering
  3. same ORDER of those keys?     no -> island/loop visit order
If all three agree and the metric still reports loss, the loss is WITHIN the
island: the seam (start vertex / direction) or arc fitting.

`--leverage` then prices it, per R692's rule that a ratio is not an effect:
it splits the per-layer in-order rate by which bucket the layer fell into. If
the order-matching layers score far higher, the gap is the fix's ceiling.

RESULT (R709, benchy classic, feature 'Outer wall'):

    300 layers; islands ours 938 vs cpp 940, count differs on 1 layer
      island SET identical  240   -> ORDER identical 166, ORDER DIFFERS 74
      island SET differs     60

    bucket           layers    rust  content in-order  in-ord%  ord loss%
    ORDER same          166   37100    35432    35028    94.4%       1.1%
    ORDER differs        74   21047    20048    13272    63.1%      33.8%

  Where the island order matches, walls are essentially clean (1.1% loss).
  Where it differs, 33.8%. Ceiling if the reordered layers scored like the
  matching ones: +6,599 in-order lines on Outer wall alone.

  Note the ordering is NOT `Layer::make_slices` -- R709's MKSL probe compared
  that chain directly against C++ and found the ordering points identical on
  155 of 156 multi-island layers and the chain order identical on all 155.

`--all` (R710) runs every feature and totals the ceiling, and prints the
per-layer/per-feature DISCRIMINATOR: on a layer, are ALL multi-island features
reordered together (consistent with one layer-level decision) or only SOME (a
per-feature decision)?

  R710, benchy classic: per-feature ceilings Outer wall +6,599, Gap infill
  +3,188, Inner wall +2,044, everything else under +50 -- TOTAL +11,911.
  The fill family scores badly even when the order MATCHES (Sparse 32.3%,
  Internal solid 27.8%, Bridge 12.4%), so their defect is geometry, not
  ordering; walls and gap infill are the ordering-actionable part.
  Discriminator, of 194 layers with >=2 comparable features: 95 all in order,
  50 all reordered, 49 mixed -- genuinely split, so neither a layer-level nor a
  per-feature cause is eliminated.

  Beware comparing raw permutation TUPLES across features: features have
  different island counts, so (0,1) vs (2,3,0,1,4) "differ" even when both are
  the identity. R710 first reported 93-vs-6 from exactly that mistake. Compare
  the BOOLEAN "is this feature reordered", not the tuples.

Usage: island_order.py <rust.gcode> <cpp.gcode> [feature] [res_mm] [--leverage]
       island_order.py <rust.gcode> <cpp.gcode> --all [res_mm]
"""
import re
import sys
from collections import Counter, defaultdict
from difflib import SequenceMatcher

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from line_parity import DEC, groups, quantise  # noqa: E402

XY = re.compile(r'\b([XY])(-?[\d.]+)')
EE = re.compile(r'\bE(-?[\d.]+)')
RES = 0.1


def islands(lines, res=RES):
    """[(bbox_key, n_moves)] in emission order."""
    runs, cur = [], []
    x = y = None
    for s in lines:
        t = s.strip()
        if not t[:2] in ('G1', 'G0', 'G2', 'G3'):
            continue
        d = {m.group(1): float(m.group(2)) for m in XY.finditer(t)}
        e = EE.search(t)
        moved = 'X' in d or 'Y' in d
        ext = e is not None and float(e.group(1)) > 0 and moved
        if 'X' in d:
            x = d['X']
        if 'Y' in d:
            y = d['Y']
        if ext:
            if x is not None and y is not None:
                cur.append((x, y))
        elif moved and cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)

    out = []
    for pts in runs:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        out.append(((round(min(xs) / res), round(min(ys) / res),
                     round(max(xs) / res), round(max(ys) / res)), len(pts)))
    return out


def classify(rraw, craw, res):
    rk = [k for k, _ in islands(rraw, res)]
    ck = [k for k, _ in islands(craw, res)]
    if Counter(rk) != Counter(ck):
        return 'SET differs', rk, ck
    return ('ORDER same' if rk == ck else 'ORDER differs'), rk, ck


FEATS = ['Outer wall', 'Inner wall', 'Gap infill', 'Sparse infill',
         'Internal solid infill', 'Top surface', 'Bottom surface',
         'Floating vertical shell', 'Overhang wall', 'Bridge']


def run_all(rg, cg, res):
    """Per-feature ceilings + the layer-vs-feature discriminator (R710)."""
    bucket = defaultdict(lambda: defaultdict(Counter))
    reordered = defaultdict(dict)      # layer -> feat -> is_in_order

    for key in sorted(set(rg) | set(cg)):
        layer, feat = key
        if feat not in FEATS:
            continue
        rraw, craw = rg.get(key, []), cg.get(key, [])
        if not rraw or not craw:
            continue
        b, rk, ck = classify(rraw, craw, res)
        if len(rk) > 1 and b != 'SET differs':
            reordered[layer][feat] = (b == 'ORDER same')
        r = [quantise(x, DEC) for x in rraw]
        c = [quantise(x, DEC) for x in craw]
        d = bucket[feat][b]
        d['layers'] += 1
        d['rust'] += len(r)
        d['content'] += sum((Counter(r) & Counter(c)).values())
        d['in_order'] += sum(bl.size for bl in SequenceMatcher(
            None, r, c, autojunk=False).get_matching_blocks())

    print(f"{'feature':<26} {'bucket':<14} {'layers':>6} {'rust':>8} "
          f"{'in-order':>9} {'in-ord%':>8}")
    total = 0
    for feat in FEATS:
        bs = bucket.get(feat)
        if not bs:
            continue
        for b in ('ORDER same', 'ORDER differs', 'SET differs'):
            d = bs.get(b)
            if d:
                print(f"{feat:<26} {b:<14} {d['layers']:>6} {d['rust']:>8} "
                      f"{d['in_order']:>9} "
                      f"{100*d['in_order']/max(d['rust'],1):>7.1f}%")
        same, diff = bs.get('ORDER same'), bs.get('ORDER differs')
        if same and diff and same['rust'] and diff['rust']:
            good = same['in_order'] / same['rust']
            ceil = min(good * diff['rust'] - diff['in_order'],
                       diff['content'] - diff['in_order'])
            if ceil > 0:
                total += ceil
                print(f"{'':<26} {'CEILING':<14} {'':>6} {'':>8} "
                      f"{'+' + str(int(ceil)):>9}")
    print(f"\nTOTAL island-order ceiling across features: +{int(total)} in-order lines")

    # Discriminator -- compare the BOOLEAN, never the permutation tuples.
    all_ok = all_re = mixed = 0
    for layer, feats in reordered.items():
        if len(feats) < 2:
            continue
        vals = list(feats.values())
        if all(vals):
            all_ok += 1
        elif not any(vals):
            all_re += 1
        else:
            mixed += 1
    print(f"\nDISCRIMINATOR over {all_ok + all_re + mixed} layers with >=2 "
          f"comparable features:")
    print(f"  ALL features in order  : {all_ok}")
    print(f"  ALL features reordered : {all_re}  (consistent with ONE layer-level decision)")
    print(f"  MIXED                  : {mixed}  (a per-FEATURE decision)")


def main():
    argv = [a for a in sys.argv[1:] if a not in ('--leverage', '--all')]
    leverage = '--leverage' in sys.argv
    rp, cp = argv[0], argv[1]
    want = argv[2] if len(argv) > 2 else 'Outer wall'
    res = float(argv[3]) if len(argv) > 3 else RES
    rg, cg = groups(rp), groups(cp)

    if '--all' in sys.argv:
        res = float(argv[2]) if len(argv) > 2 else RES
        print(f"rust={rp}\ncpp ={cp}\nbbox resolution={res} mm\n")
        run_all(rg, cg, res)
        return

    stats = Counter()
    buckets = defaultdict(Counter)
    examples = []
    layers = 0

    for key in sorted(set(rg) | set(cg)):
        layer, feat = key
        if feat != want:
            continue
        rraw, craw = rg.get(key, []), cg.get(key, [])
        if not rraw or not craw:
            continue
        layers += 1
        bucket, rk, ck = classify(rraw, craw, res)
        stats[bucket] += 1
        if len(rk) != len(ck):
            stats['count differs'] += 1
        stats['our islands'] += len(rk)
        stats['cpp islands'] += len(ck)
        if bucket == 'ORDER differs' and len(examples) < 6:
            pos = defaultdict(list)
            for i, k in enumerate(ck):
                pos[k].append(i)
            used = defaultdict(int)
            seq = []
            for k in rk:
                lst = pos[k]
                seq.append(lst[min(used[k], len(lst) - 1)])
                used[k] += 1
            examples.append((layer, len(rk), seq[:12]))

        if leverage:
            r = [quantise(x, DEC) for x in rraw]
            c = [quantise(x, DEC) for x in craw]
            d = buckets[bucket]
            d['layers'] += 1
            d['rust'] += len(r)
            d['content'] += sum((Counter(r) & Counter(c)).values())
            d['in_order'] += sum(b.size for b in SequenceMatcher(
                None, r, c, autojunk=False).get_matching_blocks())

    print(f"rust={rp}\ncpp ={cp}\nfeature={want!r}  bbox resolution={res} mm\n")
    print(f"layers with this feature on both sides: {layers}")
    print(f"  island count differs : {stats['count differs']}")
    print(f"  island SET differs   : {stats['SET differs']}")
    print(f"  island SET identical : {stats['ORDER same'] + stats['ORDER differs']}")
    print(f"     ORDER identical   : {stats['ORDER same']}")
    print(f"     ORDER differs     : {stats['ORDER differs']}")
    print(f"  total islands: ours {stats['our islands']}  cpp {stats['cpp islands']}")

    if leverage and buckets:
        print(f"\n{'bucket':<15} {'layers':>7} {'rust':>9} {'content':>9} "
              f"{'in-order':>9} {'in-ord%':>8} {'ord loss%':>10}")
        rates = {}
        for b, d in buckets.items():
            rate = 100 * d['in_order'] / max(d['rust'], 1)
            loss = 100 * (d['content'] - d['in_order']) / max(d['content'], 1)
            rates[b] = (rate, d)
            print(f"{b:<15} {d['layers']:>7} {d['rust']:>9} {d['content']:>9} "
                  f"{d['in_order']:>9} {rate:>7.1f}% {loss:>9.1f}%")
        if 'ORDER same' in rates and 'ORDER differs' in rates:
            good = rates['ORDER same'][0] / 100
            d = rates['ORDER differs'][1]
            ceil = min(good * d['rust'] - d['in_order'], d['content'] - d['in_order'])
            print(f"\nCEILING if the reordered layers scored like the matching ones:"
                  f"\n  {d['in_order']} -> {d['in_order'] + int(ceil)} in-order "
                  f"(+{int(ceil)} lines), capped by their content match {d['content']}")

    if examples:
        print("\nORDER-DIFFERING LAYERS (layer, n islands, our islands' cpp positions)")
        for e in examples:
            print(f"  layer {e[0]:>4}  n={e[1]:<4} {e[2]}")


if __name__ == '__main__':
    main()
