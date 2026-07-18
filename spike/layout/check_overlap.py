#!/usr/bin/env python3
"""spike/layout/check_overlap.py — DISPOSABLE materialization spot-check (issue #7 spike).

Reconstructs placed polygons from a layout_spike output JSON + its fixture and
verifies via SAT (convex polygons; holes are dropped by the kernel by design):
  - no pairwise overlap among placements
  - no placement intersects a locked item or an exclusion polygon

usage: check_overlap.py <output.json> <fixture.json>
exit 0 = clean, exit 1 = overlap found, exit 2 = bad input.
"""

import json
import math
import sys

EPS = 1e-3  # mm tolerance; touching edges are not an overlap


def transform(footprint, x, y, yaw_deg):
    # Kernel semantics (verified experimentally, both engines): each footprint
    # point is rotated by yaw about the footprint's frame origin, then
    # translated by (x, y). No centroid re-referencing.
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return [(px * c - py * s + x, px * s + py * c + y) for px, py in footprint]


def project(poly, ax, ay):
    dots = [px * ax + py * ay for px, py in poly]
    return min(dots), max(dots)


def overlaps(a, b):
    """SAT: True if convex polys a and b intersect with area > EPS."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            ax, ay = -(y2 - y1), (x2 - x1)  # edge normal
            norm = math.hypot(ax, ay)
            if norm == 0:
                continue
            ax, ay = ax / norm, ay / norm
            amin, amax = project(a, ax, ay)
            bmin, bmax = project(b, ax, ay)
            if amax <= bmin + EPS or bmax <= amin + EPS:
                return False  # separating axis found
    return True


def main():
    if len(sys.argv) != 3:
        print("usage: check_overlap.py <output.json> <fixture.json>", file=sys.stderr)
        return 2
    out = json.load(open(sys.argv[1]))
    fix = json.load(open(sys.argv[2]))

    items = {it["id"]: it for it in fix["items"]}
    placed = []
    for p in out["placements"]:
        fp = items[p["id"]]["footprint"]
        placed.append((p["id"], transform(fp, p["x_mm"], p["y_mm"], p["yaw_deg"])))

    obstacles = []
    for lk in out["locked"]:
        it = items[lk["id"]]
        obstacles.append((lk["id"], transform(it["footprint"], lk["x_mm"], lk["y_mm"], lk["yaw_deg"])))
    for ex in fix.get("exclusions", []):
        tx, ty = ex.get("translation", [0, 0])
        obstacles.append((ex.get("id", "exclusion"),
                          transform(ex["polygon"], tx, ty, ex.get("rotation_deg", 0))))

    bad = []
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            if overlaps(placed[i][1], placed[j][1]):
                bad.append(f"placement overlap: {placed[i][0]} x {placed[j][0]}")
        for oid, opoly in obstacles:
            if overlaps(placed[i][1], opoly):
                bad.append(f"placement {placed[i][0]} intersects obstacle {oid}")

    if bad:
        for b in bad:
            print("OVERLAP:", b)
        return 1
    print(f"OK: {len(placed)} placements, {len(obstacles)} obstacles, no overlaps")
    return 0


if __name__ == "__main__":
    sys.exit(main())
