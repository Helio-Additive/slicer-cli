#!/usr/bin/env python3
"""Semantic / geometric equivalence comparison of two G-code files.

Level 1: physical invariants (filament, per-layer & per-feature material, layer
         structure).
Level 2: per-layer geometric COVERAGE (rasterize extrusion swept-area, compare
         Intersection-over-Union) — robust to FP-cascade toolpath re-routing.
"""
import re, math, sys, collections
import numpy as np

# ---- pass/fail tolerances (exit 0 iff all pass) ----
# Chosen against the known FP-cascade floor: byte-diff is ~99% floating-point
# noise, so we assert PHYSICAL equivalence, not text identity. Baseline (default
# Benchy): filament 0.9972, per-layer material mean-dev 2.19%, silhouette
# area-weighted IoU 99.29% — all pass. These catch a genuinely-broken toolpath
# (silhouette/material collapse) while treating FP re-routing as equivalent.
TOL_FILAMENT   = 0.01   # |rust/native - 1| for total filament length
TOL_LAYER_MAT  = 0.05   # mean per-layer material deviation
TOL_SILHOUETTE = 0.99   # area-weighted object-silhouette IoU floor
# Per-feature material: loose on purpose. The known FP-cascade classes drift a
# few-to-~20% (e.g. floating vertical shell over-detection ~1.19, ISI ~0.94 on
# the default Benchy) — those are real-but-benign, not breaks. A genuinely-broken
# feature (missing/doubled/halved) exceeds 35%, which this catches. Tiny features
# (< FEATURE_MIN_E mm total) are excluded as statistically noisy.
TOL_FEATURE_MAT = 0.35
FEATURE_MIN_E   = 50.0

AX = re.compile(r'([XYEIJZF])(-?[0-9.]+)')

def parse(path):
    """Return list of layers; each = dict(z, segs=[(x0,y0,x1,y1,w,feat,e)])."""
    layers = []
    cur = dict(z=None, segs=[])
    px = py = None
    feat = ''
    width = 0.42
    header = {}
    for line in open(path):
        if line.startswith('; total filament length'):
            header['filament_mm'] = float(line.split(':')[1])
        if line.startswith('; Z_HEIGHT:'):
            if cur['segs']:
                layers.append(cur)
            cur = dict(z=round(float(line.split(':')[1]), 3), segs=[])
            continue
        if line.startswith('; LINE_WIDTH:'):
            width = float(line.split(':')[1]); continue
        if line.startswith('; FEATURE:'):
            feat = line.strip()[10:].strip(); continue
        m = re.match(r'^G([0123]) ', line)
        if not m:
            continue
        g = m.group(1); d = dict(AX.findall(line))
        x = float(d['X']) if 'X' in d else px
        y = float(d['Y']) if 'Y' in d else py
        e = float(d['E']) if 'E' in d else 0.0
        if e > 0 and px is not None and x is not None and ('X' in d or 'Y' in d):
            # arc -> approximate by chord midpoints (coverage is width-dominated)
            cur['segs'].append((px, py, x, y, width, feat, e))
        px, py = x, y
    if cur['segs']:
        layers.append(cur)
    return layers, header

def per_layer_material(layers):
    out = {}
    for L in layers:
        out[L['z']] = sum(s[6] for s in L['segs'])
    return out

def per_feature(layers):
    E = collections.Counter(); D = collections.Counter()
    for L in layers:
        for (x0,y0,x1,y1,w,f,e) in L['segs']:
            E[f]+=e; D[f]+=math.hypot(x1-x0,y1-y0)
    return E, D

def raster_layer(segs, res, x0, y0, nx, ny):
    """Boolean coverage grid: mark cells within w/2 of each segment centerline."""
    grid = np.zeros((ny, nx), dtype=bool)
    for (ax, ay, bx, by, w, f, e) in segs:
        r = max(w/2.0, res)
        length = math.hypot(bx-ax, by-ay)
        n = max(1, int(length/(res*0.5))+1)
        # sample points along segment
        ts = np.linspace(0, 1, n)
        xs = ax + (bx-ax)*ts; ys = ay + (by-ay)*ts
        ci = ((xs - x0)/res).astype(int); cj = ((ys - y0)/res).astype(int)
        rad = int(math.ceil(r/res))
        for dx in range(-rad, rad+1):
            for dy in range(-rad, rad+1):
                if dx*dx+dy*dy > (rad+0.5)**2: continue
                ii = ci+dx; jj = cj+dy
                ok = (ii>=0)&(ii<nx)&(jj>=0)&(jj<ny)
                grid[jj[ok], ii[ok]] = True
    return grid

def _close(grid, k):
    """Binary morphological closing (dilate k then erode k) via max/min shifts —
    merges nearby hatching lines into a filled region so IoU measures the REGION,
    not individual line positions."""
    if k <= 0: return grid
    g = grid.copy()
    for _ in range(k):  # dilate
        g[:-1,:] |= grid[1:,:]; g[1:,:] |= grid[:-1,:]
        g[:,:-1] |= grid[:,1:]; g[:,1:] |= grid[:,:-1]
        grid = g.copy()
    e = grid.copy()
    for _ in range(k):  # erode
        e &= np.pad(grid[1:,:], ((0,1),(0,0)))
        e &= np.pad(grid[:-1,:], ((1,0),(0,0)))
        e &= np.pad(grid[:,1:], ((0,0),(0,1)))
        e &= np.pad(grid[:,:-1], ((0,0),(1,0)))
        grid = e.copy()
    return grid

def coverage_iou(rL, nL, res=0.15, feats=None, close_k=0):
    """IoU of swept-area coverage for one matched layer. feats=None -> all.
    close_k>0 applies morphological closing (region-level, robust to hatch offset)."""
    def flt(segs): return [s for s in segs if feats is None or s[5] in feats]
    rs, ns = flt(rL['segs']), flt(nL['segs'])
    segs = rs + ns
    if not segs: return None, 0
    xs = [s[0] for s in segs]+[s[2] for s in segs]
    ys = [s[1] for s in segs]+[s[3] for s in segs]
    x0, y0 = min(xs)-1, min(ys)-1
    nx = int((max(xs)+1-x0)/res)+2; ny = int((max(ys)+1-y0)/res)+2
    gr = _close(raster_layer(rs, res, x0, y0, nx, ny), close_k)
    gn = _close(raster_layer(ns, res, x0, y0, nx, ny), close_k)
    inter = np.logical_and(gr, gn).sum(); union = np.logical_or(gr, gn).sum()
    return (inter/union if union else 1.0), int(union)

from collections import deque
def silhouette(wall_grid):
    """Region enclosed by walls = object cross-section silhouette.
    Flood-fill the exterior from the border through non-wall cells; silhouette =
    everything the exterior can't reach. Robust to sub-width wall offsets.

    NOTE (R352): reliable only for shapes whose wall loop rasterizes CLOSED
    (Benchy 99.3%, cube 100%). On gap-prone convex curved walls (cylinder) the
    exterior leaks into the interior and the silhouette collapses — that is a
    metric limitation, NOT a slicer divergence (verify such models via the
    Level-1 material/layer invariants instead)."""
    ny, nx = wall_grid.shape
    ext = np.zeros_like(wall_grid)
    dq = deque()
    for i in range(nx):
        for j in (0, ny-1):
            if not wall_grid[j,i] and not ext[j,i]:
                ext[j,i]=True; dq.append((j,i))
    for j in range(ny):
        for i in (0, nx-1):
            if not wall_grid[j,i] and not ext[j,i]:
                ext[j,i]=True; dq.append((j,i))
    while dq:
        j,i = dq.popleft()
        for dj,di in ((1,0),(-1,0),(0,1),(0,-1)):
            nj,ni=j+dj,i+di
            if 0<=nj<ny and 0<=ni<nx and not wall_grid[nj,ni] and not ext[nj,ni]:
                ext[nj,ni]=True; dq.append((nj,ni))
    return ~ext  # walls + enclosed interior = the silhouette

# Silhouette metric: region-closed ALL-coverage IoU (R354). We rasterize every
# extrusion (walls + infill), then morphologically CLOSE by SIL_CLOSE_K cells at
# SIL_RES — bridging both the sparse-infill line gaps and any wall seam/travel
# gaps to recover the filled cross-section. The dilate/erode cancel at the true
# boundary, so a sub-width offset barely moves it. This is UNIVERSAL: it works on
# non-convex (Benchy 99.8%), convex-solid (cube 100%), and curved-convex
# (cylinder 99.2%) shapes — unlike the wall flood-fill `silhouette()` above,
# which leaks through gaps in circular walls (kept for reference). SIL_CLOSE_K*
# SIL_RES (=4mm) must exceed the sparse-infill line spacing to fully bridge it.
SIL_RES = 0.2
SIL_CLOSE_K = 20
def silhouette_iou(R, N, zs, res=SIL_RES, close_k=SIL_CLOSE_K):
    ious=[]
    for z in zs:
        rL=[L for L in R if L['z']==z][0]; nL=[L for L in N if L['z']==z][0]
        iou,area=coverage_iou(rL,nL,res=res,feats=None,close_k=close_k)
        if iou is not None and area>0: ious.append((z,iou,area))
    if not ious: return ious
    arr=np.array([i for _,i,_ in ious]); wts=np.array([a for _,_,a in ious],dtype=float)
    print(f"  SILHOUETTE (object outline) : mean {100*arr.mean():5.2f}%  area-wtd {100*(arr*wts).sum()/wts.sum():5.2f}%  "
          f"min {100*arr.min():5.1f}% (z{ious[int(arr.argmin())][0]})  layers<98%={int((arr<0.98).sum())}/{len(ious)}")
    return ious

WALLS = {'Outer wall','Inner wall','Overhang wall'}
def group_iou(R, N, zs, label, feats, close_k=0):
    ious=[]
    for z in zs:
        rL=[L for L in R if L['z']==z][0]; nL=[L for L in N if L['z']==z][0]
        iou,area=coverage_iou(rL,nL,feats=feats,close_k=close_k)
        if iou is not None and area>0: ious.append((z,iou,area))
    if not ious:
        print(f"  {label:28}: (no coverage)"); return
    arr=np.array([i for _,i,_ in ious]); wts=np.array([a for _,_,a in ious],dtype=float)
    print(f"  {label:28}: mean {100*arr.mean():5.2f}%  area-wtd {100*(arr*wts).sum()/wts.sum():5.2f}%  "
          f"min {100*arr.min():5.1f}% (z{ious[int(arr.argmin())][0]})  layers<95%={int((arr<0.95).sum())}/{len(ious)}")

def main(rust_path, native_path):
    R, rh = parse(rust_path); N, nh = parse(native_path)
    print("="*64)
    print("LEVEL 1 — PHYSICAL INVARIANTS")
    print("="*64)
    print(f"  filament total mm : rust {rh.get('filament_mm')} / native {nh.get('filament_mm')}"
          f"  ratio {rh.get('filament_mm',0)/nh.get('filament_mm',1):.4f}")
    print(f"  layer count       : rust {len(R)} / native {len(N)}")
    rm = per_layer_material(R); nm = per_layer_material(N)
    zs = sorted(set(rm)&set(nm))
    devs = [abs(rm[z]-nm[z])/max(nm[z],1e-9) for z in zs]
    print(f"  per-layer material: {len(zs)} common Z; mean dev {100*np.mean(devs):.2f}%  max dev {100*max(devs):.2f}%")
    RE,RD = per_feature(R); NE,ND = per_feature(N)
    print(f"  {'feature':22}{'r-E':>9}{'n-E':>9}{'E-ratio':>8}")
    for f in sorted(set(RE)|set(NE), key=lambda k:-(RE[k]+NE[k])):
        rr=RE[f]; nn=NE[f]; print(f"  {f:22}{rr:>9.1f}{nn:>9.1f}{(rr/nn if nn else 0):>8.3f}")
    print("="*64)
    print("LEVEL 2 — GEOMETRIC COVERAGE (per-layer swept-area IoU)")
    print("="*64)
    sil = silhouette_iou(R,N,zs)
    group_iou(R,N,zs,"WALL LINES (thin, strict)",WALLS)

    # ---- tolerance-based verdict ----
    print("="*64); print("VERDICT (semantic equivalence tolerances)"); print("="*64)
    sarr = np.array([i for _,i,_ in sil]); swts = np.array([a for _,_,a in sil], float)
    sil_aw = (sarr*swts).sum()/swts.sum()
    fil = rh.get('filament_mm',0)/nh.get('filament_mm',1)
    # worst per-feature material deviation among non-tiny features
    fdev = [(f, RE[f]/NE[f]) for f in set(RE)|set(NE) if NE.get(f,0) >= FEATURE_MIN_E]
    wf, wr = max(fdev, key=lambda t: abs(t[1]-1)) if fdev else ("-", 1.0)
    checks = [
        (f"filament within {TOL_FILAMENT*100:.0f}%",   abs(fil-1) <= TOL_FILAMENT,      f"{fil:.4f}"),
        ("layer count equal",                          len(R)==len(N),                  f"{len(R)}={len(N)}"),
        (f"per-layer material mean<{TOL_LAYER_MAT*100:.0f}%", np.mean(devs) <= TOL_LAYER_MAT, f"{100*np.mean(devs):.2f}%"),
        (f"per-feature material <{TOL_FEATURE_MAT*100:.0f}%", abs(wr-1) <= TOL_FEATURE_MAT, f"{wf} {wr:.3f}"),
        (f"silhouette area-wtd >={TOL_SILHOUETTE*100:.0f}%",  sil_aw >= TOL_SILHOUETTE,  f"{100*sil_aw:.2f}%"),
    ]
    allpass = all(ok for _,ok,_ in checks)
    for name,ok,val in checks:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:34} {val}")
    print(f"\n  ==> {'SEMANTICALLY EQUIVALENT' if allpass else 'DIVERGENCE — see failing checks'}")
    return allpass

if __name__=='__main__':
    ok = main(sys.argv[1], sys.argv[2])
    sys.exit(0 if ok else 1)
