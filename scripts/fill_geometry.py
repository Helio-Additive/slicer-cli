#!/usr/bin/env python3
"""R711: WHY do the fills diverge? Angle, spacing, grid phase, direction, or
the clipping boundary?

R710 proved the fill family's defect is geometry rather than ordering: on
layers where the island order matches C++ exactly, Sparse still scores 32.3%
in-order, Internal solid 27.8%, Floating vertical shell 15.7%, Bridge 12.4% --
while walls on those same layers score 87-94%. That says "the fills are wrong",
not which part of them.

This tests the fill pipeline's parameters one at a time, straight from the
emitted G-code -- no build, no probe. Every stage carries `Outer wall` as a
CONTROL: a measurement that cannot show the walls matching is measuring itself.

  1. ANGLE      histogram of extrusion-segment directions, folded to [0,180)
  2. SPACING    median gap between consecutive perpendicular offsets
  3. PHASE      the first perpendicular offset -- the grid's origin
  4. DIRECTION  segments matching exactly vs matching REVERSED
  5. VERTICES   how many extrusion endpoints are shared at all

RESULT (R711, benchy classic):

  angle    Sparse 45/135/90 at 20/20/19% on BOTH sides; sets identical
  spacing  identical (5.54 mm sparse, 0.39 mm solid)
  phase    identical (shift 0.000 on nearly every sampled layer)
  length   total extruded length within 0.3-1.6% on every fill feature
  direction  reversal explains almost nothing (sparse 9.3%, solid 3.7%,
             bridge and floating shell 0.0%)
  vertices SHARED: Outer wall 99.5% -- Sparse 41.8%, Top 19.0%,
           Internal solid 14.0%, Floating vertical shell 4.1%, Bridge 2.7%

  So the fill lines lie on the SAME grid and cover the SAME total length, yet
  their endpoints differ. The pattern parameters are faithful; what differs is
  where each line is CLIPPED -- i.e. the surface handed to the fill. Four
  hypotheses (angle, spacing, phase, direction) died here; the boundary is the
  target.

Usage: fill_geometry.py <rust.gcode> <cpp.gcode> [feature ...]
"""
import math
import re
import sys
from collections import Counter, defaultdict

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from line_parity import groups  # noqa: E402

XY = re.compile(r'\b([XY])(-?[\d.]+)')
EE = re.compile(r'\bE(-?[\d.]+)')
MIN_LEN = 0.5        # mm; shorter segments have meaningless angles
DEFAULT_FEATS = ['Sparse infill', 'Internal solid infill', 'Top surface',
                 'Bottom surface', 'Bridge', 'Floating vertical shell',
                 'Outer wall']
CONTROL = 'Outer wall'


def extrusions(lines):
    """[(start, end, angle_deg, length)] for every extruding XY move."""
    out = []
    x = y = None
    for s in lines:
        t = s.strip()
        if t[:2] not in ('G1', 'G0'):
            continue
        d = {m.group(1): float(m.group(2)) for m in XY.finditer(t)}
        e = EE.search(t)
        nx, ny = d.get('X', x), d.get('Y', y)
        ext = e is not None and float(e.group(1)) > 0 and ('X' in d or 'Y' in d)
        if ext and None not in (x, y, nx, ny):
            dx, dy = nx - x, ny - y
            L = math.hypot(dx, dy)
            if L >= MIN_LEN:
                out.append(((round(x, 3), round(y, 3)), (round(nx, 3), round(ny, 3)),
                            math.degrees(math.atan2(dy, dx)) % 180.0, L))
        x, y = nx, ny
    return out


def endpoints(lines):
    """Every extrusion endpoint, for the vertex-overlap test."""
    out = []
    x = y = None
    for s in lines:
        t = s.strip()
        if t[:2] not in ('G1', 'G0'):
            continue
        d = {m.group(1): float(m.group(2)) for m in XY.finditer(t)}
        e = EE.search(t)
        nx, ny = d.get('X', x), d.get('Y', y)
        if (e is not None and float(e.group(1)) > 0 and ('X' in d or 'Y' in d)
                and nx is not None and ny is not None):
            out.append((round(nx, 3), round(ny, 3)))
        x, y = nx, ny
    return out


def phase(segs, want_deg, tol=2.0):
    """Perpendicular offsets of segments running at `want_deg`."""
    a = math.radians(want_deg)
    sa, ca = math.sin(a), math.cos(a)
    out = []
    for (x, y), _, ang, _ in segs:
        if min(abs(ang - want_deg), 180 - abs(ang - want_deg)) <= tol:
            out.append(round(x * sa - y * ca, 3))
    return sorted(out)


def main():
    rp, cp = sys.argv[1], sys.argv[2]
    feats = sys.argv[3:] or DEFAULT_FEATS
    if CONTROL not in feats:
        feats = list(feats) + [CONTROL]
    rg, cg = groups(rp), groups(cp)

    per = {}
    for feat in feats:
        R, C, RP, CP = [], [], [], []
        rl, cl = defaultdict(list), defaultdict(list)
        for key in set(rg) | set(cg):
            layer, f = key
            if f != feat:
                continue
            rs, cs = extrusions(rg.get(key, [])), extrusions(cg.get(key, []))
            R += rs
            C += cs
            rl[layer] += rs
            cl[layer] += cs
            RP += endpoints(rg.get(key, []))
            CP += endpoints(cg.get(key, []))
        if R and C:
            per[feat] = (R, C, RP, CP, rl, cl)

    print("1. ANGLE distribution (folded to [0,180), >=%.1f mm segments)" % MIN_LEN)
    for feat, (R, C, _, _, _, _) in per.items():
        rh = Counter(int(round(a)) % 180 for _, _, a, _ in R)
        ch = Counter(int(round(a)) % 180 for _, _, a, _ in C)
        rs = {a for a, n in rh.items() if n >= 0.02 * len(R)}
        cs = {a for a, n in ch.items() if n >= 0.02 * len(C)}
        verdict = 'SETS MATCH' if rs == cs else f'differ (ours-only {sorted(rs-cs)}, cpp-only {sorted(cs-rs)})'
        print(f"  {feat:<26} {verdict}")

    print("\n2/3. SPACING and PHASE of the 45deg family, per layer (first 4 shown)")
    for feat, (_, _, _, _, rl, cl) in per.items():
        shown = 0
        for layer in sorted(set(rl) & set(cl)):
            r, c = phase(rl[layer], 45), phase(cl[layer], 45)
            if len(r) < 4 or len(c) < 4:
                continue
            ur = sorted(set(round(v, 2) for v in r))
            uc = sorted(set(round(v, 2) for v in c))
            dr = sorted(round(ur[i+1]-ur[i], 3) for i in range(len(ur)-1))
            dc = sorted(round(uc[i+1]-uc[i], 3) for i in range(len(uc)-1))
            print(f"  {feat:<24} L{layer:<4} spacing {dr[len(dr)//2]}/{dc[len(dc)//2]} "
                  f"phase {ur[0]}/{uc[0]} shift {round(ur[0]-uc[0], 3)}")
            shown += 1
            if shown >= 4:
                break

    print("\n4. DIRECTION: segments matching exactly vs REVERSED")
    print(f"  {'feature':<26} {'ours':>7} {'same':>8} {'reversed':>9} {'neither':>9}")
    for feat, (R, C, _, _, _, _) in per.items():
        rr = Counter((a, b) for a, b, _, _ in R)
        cf = Counter((a, b) for a, b, _, _ in C)
        cr = Counter((b, a) for a, b, _, _ in C)
        same = sum((rr & cf).values())
        rev = sum((rr & cr).values())
        print(f"  {feat:<26} {len(R):>7} {100*same/len(R):>7.1f}% "
              f"{100*rev/len(R):>8.1f}% {100*(len(R)-same-rev)/len(R):>8.1f}%")

    print("\n5. VERTEX overlap, and TOTAL LENGTH ratio")
    print(f"  {'feature':<26} {'our pts':>8} {'cpp pts':>8} {'shared':>8} "
          f"{'%ours':>7} {'len ratio':>10}")
    for feat, (R, C, RP, CP, _, _) in per.items():
        sh = sum((Counter(RP) & Counter(CP)).values())
        lr = sum(L for _, _, _, L in R)
        lc = sum(L for _, _, _, L in C)
        print(f"  {feat:<26} {len(RP):>8} {len(CP):>8} {sh:>8} "
              f"{100*sh/max(len(RP),1):>6.1f}% {lr/max(lc,1e-9):>10.4f}")
    print(f"\n  ({CONTROL} is the CONTROL -- if it does not score near 100%, the"
          " measurement is broken, not the fills.)")


if __name__ == '__main__':
    main()
