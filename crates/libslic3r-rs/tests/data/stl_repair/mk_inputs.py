#!/usr/bin/env python3
"""Binary STL inputs for tests/stl_repair_native_parity.rs: meshes that need admesh's
whole import repair (TriangleMesh.cpp:102-197). Usage: mk_inputs.py <out dir>.
Deterministic (a simple LCG, no random module)."""
import math, struct, sys, os

class Lcg:
    def __init__(self, seed): self.s = seed & 0xffffffff
    def next(self):
        self.s = (self.s * 1664525 + 1013904223) & 0xffffffff
        return self.s
    def unit(self): return self.next() / 4294967296.0

def f32(x): return struct.unpack('<f', struct.pack('<f', x))[0]

def icosphere(sub, r=10.0, c=(0, 0, 0)):
    t = (1 + 5 ** 0.5) / 2
    vs = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
          (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    fs = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6),
          (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10),
          (8, 6, 7), (9, 8, 1)]
    vs = [tuple(x / math.sqrt(sum(y * y for y in v)) for x in v) for v in vs]
    for _ in range(sub):
        cache = {}
        def mid(a, b):
            k = (min(a, b), max(a, b))
            if k not in cache:
                m = [(vs[a][i] + vs[b][i]) / 2 for i in range(3)]
                l = math.sqrt(sum(x * x for x in m))
                vs.append(tuple(x / l for x in m))
                cache[k] = len(vs) - 1
            return cache[k]
        nf = []
        for a, b, cc in fs:
            ab, bc, ca = mid(a, b), mid(b, cc), mid(cc, a)
            nf += [(a, ab, ca), (b, bc, ab), (cc, ca, bc), (ab, bc, ca)]
        fs = nf
    pts = [tuple(f32(c[i] + r * v[i]) for i in range(3)) for v in vs]
    return [[pts[a], pts[b], pts[cc]] for a, b, cc in fs]

def box(x0, y0, z0, x1, y1, z1):
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    q = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6),
         (3, 0, 4), (3, 4, 7)]
    p = [tuple(f32(x) for x in v) for v in p]
    return [[p[a], p[b], p[c]] for a, b, c in q]

def normal(f):
    u = [f[1][i] - f[0][i] for i in range(3)]
    w = [f[2][i] - f[0][i] for i in range(3)]
    n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]]
    l = math.sqrt(sum(x * x for x in n)) or 1.0
    return [x / l for x in n]

def write(path, facets, normals=None):
    with open(path, 'wb') as fh:
        fh.write(b'binary stl for stl_repair_native_parity'.ljust(80, b' '))
        fh.write(struct.pack('<I', len(facets)))
        for i, f in enumerate(facets):
            n = normals[i] if normals is not None else normal(f)
            fh.write(struct.pack('<3f', *n))
            for v in f:
                fh.write(struct.pack('<3f', *v))
            fh.write(b'\0\0')

def jitter(facets, rng, mag, frac):
    out = []
    for f in facets:
        nf = []
        for v in f:
            if rng.unit() < frac:
                v = tuple(f32(x + (rng.unit() - 0.5) * 2 * mag) for x in v)
            nf.append(v)
        out.append(nf)
    return out

def main(out):
    os.makedirs(out, exist_ok=True)
    rng = Lcg(952)
    s2 = icosphere(2)
    # 1. clean sphere (manifold after the exact check)
    write(f'{out}/sphere_clean.stl', s2)
    # 2-5. jittered copies: nearby merge over one or two iterations, then remove_unconnected
    for k, (mag, frac) in enumerate([(1e-5, 0.3), (5e-5, 0.5), (2e-4, 0.5), (1e-3, 0.2)]):
        write(f'{out}/sphere_jit{k}.stl', jitter(s2, rng, mag, frac))
    # 6. sphere with a third of the facets flipped and garbage normals in the file
    fl = []
    nrm = []
    for i, f in enumerate(s2):
        if rng.unit() < 0.33:
            f = [f[1], f[0], f[2]]
        fl.append(f)
        nrm.append([f32(rng.unit() * 2 - 1) for _ in range(3)] if rng.unit() < 0.5 else normal(f))
    write(f'{out}/sphere_flip.stl', fl, nrm)
    # 7. inside-out box (negative volume -> stl_reverse_all_facets), zero normals in the file
    b = box(0, 0, 0, 20, 10, 5)
    write(f'{out}/box_inverted.stl', [[f[0], f[2], f[1]] for f in b], [[0.0, 0.0, 0.0]] * len(b))
    # 8. two boxes (two parts), the second one flipped facet by facet, plus an isolated facet and degenerate facets
    b2 = box(30, 0, 0, 40, 10, 10)
    extra = [[(f32(50), f32(0), f32(0)), (f32(51), f32(0), f32(0)), (f32(50), f32(1), f32(0))],
             [(f32(1), f32(1), f32(1)), (f32(1), f32(1), f32(1)), (f32(2), f32(2), f32(2))],
             [(f32(0), f32(0), f32(0)), (f32(5), f32(0), f32(0)), (f32(5), f32(0), f32(0))]]
    write(f'{out}/two_parts.stl', b + [[f[0], f[2], f[1]] for f in b2] + extra)
    # 9. jittered sphere with flipped facets and a NaN facet
    s1 = icosphere(1, 7.5, (3, -2, 4))
    j = jitter(s1, rng, 3e-5, 0.6)
    j = [[f[1], f[0], f[2]] if rng.unit() < 0.25 else f for f in j]
    j.insert(7, [(float('nan'), 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)])
    write(f'{out}/sphere_mix.stl', j)
    # 10. box with a slit: one edge's vertices pulled apart by the jitter so that the
    # merge collapses facets (degenerate after nearby)
    bs = box(-5, -5, -5, 5, 5, 5)
    bs = jitter(bs, rng, 2e-3, 0.5)
    sl = [[(f32(-5), f32(-5), f32(-5)), (f32(-5 + 1e-4), f32(-5), f32(-5)), (f32(-5), f32(-5 + 1e-4), f32(-5 + 1e-4))]]
    write(f'{out}/box_sliver.stl', bs + sl)

if __name__ == '__main__':
    main(sys.argv[1])
