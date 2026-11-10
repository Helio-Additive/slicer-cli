#!/usr/bin/env python3
"""R594: is the fill lattice at the same ANGLE and the same PHASE?

Same coverage + same total length + different placement (R593) admits three
causes. This separates them geometrically, per (layer, feature):

  ANGLE   histogram of segment directions mod 180 degrees. If the angles differ,
          the fill direction is wrong and nothing downstream matters.
  PHASE   for the dominant angle, project every segment midpoint onto the normal
          of that direction. Fill lines form a lattice, so the projections
          cluster at multiples of the line spacing. Compare the two engines'
          lattice offsets: a constant shift is a phase error.
  ORDER   if angle and phase both match, the SET of lines is the same and only
          the emission order/anchoring differs.

Usage: fill_geom.py <rust.gcode> <cpp.gcode> <feature> [max_layers]
"""
import sys
import re
import math
from collections import defaultdict, Counter

FIELD = re.compile(r'([A-Z])(-?\d*\.?\d+(?:[eE][-+]?\d+)?)')


def segments(path, feature):
    """{layer: [(x0,y0,x1,y1,len)]} for extruding moves inside `feature`."""
    out = defaultdict(list)
    layer, feat = 0, None
    x = y = None
    with open(path, errors='replace') as fh:
        for raw in fh:
            ln = raw.rstrip()
            if ln.startswith('; CHANGE_LAYER'):
                layer += 1
                feat = None
                continue
            if ln.startswith('; FEATURE:'):
                feat = ln.split(':', 1)[1].strip()
                continue
            if ln[:3] not in ('G1 ', 'G2 ', 'G3 '):
                continue
            f = {k: float(v) for k, v in FIELD.findall(ln)}
            nx, ny = f.get('X', x), f.get('Y', y)
            if (feat == feature and 'E' in f and f['E'] > 0
                    and x is not None and nx is not None and ny is not None):
                d = math.hypot(nx - x, ny - y)
                if d > 1e-6:
                    out[layer].append((x, y, nx, ny, d))
            x, y = nx, ny
    return out


def angle_hist(segs, bin_deg=1.0):
    h = Counter()
    for x0, y0, x1, y1, d in segs:
        a = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
        h[round(a / bin_deg) * bin_deg % 180.0] += d      # weight by length
    return h


def lattice(segs, ang_deg, tol=15.0):
    """Perpendicular offsets of segments whose direction is near `ang_deg`."""
    th = math.radians(ang_deg)
    nx, ny = -math.sin(th), math.cos(th)
    offs = []
    for x0, y0, x1, y1, d in segs:
        a = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
        if min(abs(a - ang_deg), 180 - abs(a - ang_deg)) > tol:
            continue
        mx, my = (x0 + x1) / 2, (y0 + y1) / 2
        offs.append(mx * nx + my * ny)
    return sorted(offs)


def main():
    rp, cp, feature = sys.argv[1], sys.argv[2], sys.argv[3]
    maxl = int(sys.argv[4]) if len(sys.argv) > 4 else 6
    rs, cs = segments(rp, feature), segments(cp, feature)

    layers = sorted(set(rs) & set(cs), key=lambda L: -(len(rs[L]) + len(cs[L])))[:maxl]
    print(f"feature: {feature}   comparing {len(layers)} busiest shared layers\n")

    for L in layers:
        r, c = rs[L], cs[L]
        rh, ch = angle_hist(r), angle_hist(c)
        rtop = rh.most_common(3)
        ctop = ch.most_common(3)
        print(f"layer {L}: rust {len(r)} segs / {sum(s[4] for s in r):.1f} mm   "
              f"cpp {len(c)} segs / {sum(s[4] for s in c):.1f} mm")
        print(f"  angle (deg: mm)  rust {[(a, round(v,1)) for a,v in rtop]}")
        print(f"                   cpp  {[(a, round(v,1)) for a,v in ctop]}")
        if not rtop or not ctop:
            continue
        ang = rtop[0][0]
        ro, co = lattice(r, ang), lattice(c, ang)
        if len(ro) < 3 or len(co) < 3:
            print(f"  (too few segments at {ang} deg for a lattice test)\n")
            continue
        # spacing from consecutive distinct offsets
        def spacing(o):
            u = []
            for v in o:
                if not u or v - u[-1] > 0.05:
                    u.append(v)
            g = [u[i+1] - u[i] for i in range(len(u)-1)]
            g.sort()
            return (u, g[len(g)//2] if g else 0.0)
        ru, rsp = spacing(ro)
        cu, csp = spacing(co)
        print(f"  lattice @ {ang:.0f} deg: rust {len(ru)} lines, spacing {rsp:.4f} mm | "
              f"cpp {len(cu)} lines, spacing {csp:.4f} mm")
        if rsp > 1e-6 and csp > 1e-6:
            rph = [v % rsp for v in ru]
            cph = [v % csp for v in cu]
            rph.sort(); cph.sort()
            rm = rph[len(rph)//2]
            cm = cph[len(cph)//2]
            print(f"  lattice PHASE (median offset mod spacing): rust {rm:.4f}  cpp {cm:.4f}  "
                  f"delta {abs(rm-cm):.4f} mm")
        # nearest-neighbour offset match
        matched = 0
        cc = list(cu)
        for v in ru:
            if any(abs(v - w) <= 0.001 for w in cc):
                matched += 1
        print(f"  lattice lines with a counterpart within 1um: {matched}/{len(ru)} "
              f"({100*matched/max(len(ru),1):.1f}%)\n")


if __name__ == '__main__':
    main()
