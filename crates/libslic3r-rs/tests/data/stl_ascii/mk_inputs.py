# Writes the ASCII STL inputs of stl_ascii_native_parity (python3 mk_inputs.py <out dir>).
import math, sys, os
out = sys.argv[1]

def tube(n=24, R=10.0, r=4.0, h=3.0):
    tris = []
    for i in range(n):
        a0 = 2*math.pi*i/n; a1 = 2*math.pi*(i+1)/n
        o0 = (R*math.cos(a0), R*math.sin(a0)); o1 = (R*math.cos(a1), R*math.sin(a1))
        i0 = (r*math.cos(a0), r*math.sin(a0)); i1 = (r*math.cos(a1), r*math.sin(a1))
        tris += [((o0[0],o0[1],0), (o1[0],o1[1],0), (o1[0],o1[1],h)), ((o0[0],o0[1],0), (o1[0],o1[1],h), (o0[0],o0[1],h))]
        tris += [((i0[0],i0[1],0), (i1[0],i1[1],h), (i1[0],i1[1],0)), ((i0[0],i0[1],0), (i0[0],i0[1],h), (i1[0],i1[1],h))]
        tris += [((i0[0],i0[1],h), (o0[0],o0[1],h), (o1[0],o1[1],h)), ((i0[0],i0[1],h), (o1[0],o1[1],h), (i1[0],i1[1],h))]
        tris += [((i0[0],i0[1],0), (o1[0],o1[1],0), (o0[0],o0[1],0)), ((i0[0],i0[1],0), (i1[0],i1[1],0), (o1[0],o1[1],0))]
    return tris

def cube(s=25.6):
    v = [(x*s, y*s, z*s) for x in (0,1) for y in (0,1) for z in (0,1)]
    q = [(0,1,3,2), (4,6,7,5), (0,4,5,1), (2,3,7,6), (0,2,6,4), (1,5,7,3)]
    tris = []
    for a,b,c,d in q:
        tris += [(v[a],v[b],v[c]), (v[a],v[c],v[d])]
    return tris

def write(name, tris, fmt="%.6f", nl="\n", vsep=" ", extra=None, normal="0 0 0", vtrail=""):
    with open(os.path.join(out, name), "w", newline="") as f:
        f.write("solid " + name + nl)
        for k, t in enumerate(tris):
            if extra and k in extra:
                f.write(extra[k])
            f.write("  facet normal " + normal + nl + "    outer loop" + nl)
            for p in t:
                f.write("      vertex" + vsep + vsep.join(fmt % c for c in p) + vtrail + nl)
            f.write("    endloop" + nl + "  endfacet" + nl)
        f.write("endsolid " + name + nl)

write("tube24_f6.stl", tube())
# %e keeps sin(2 pi) = -2.4e-16 apart from sin(0) = 0: snap them to zero so the exact check connects
# the mesh (tube24_e_seam.stl keeps them: only admesh's stl_check_facets_nearby closes that seam)
snap = [tuple(tuple(0.0 if abs(c) < 1e-9 else c for c in p) for p in t) for t in tube()]
write("tube24_e_crlf.stl", snap, fmt="%e", nl="\r\n", vsep="\t",
      extra={10: "endsolid part\r\n\r\nsolid part two\r\n"}, normal="-0.0 nan 1e-3")
write("tube24_e_seam.stl", tube(), fmt="%e")
c = cube()
write("cube_long3.stl", c, vtrail=" " * 110 if False else "", extra=None)
# three vertex lines longer than 99 characters: the count grows by 3 (still 12 facets)
lines = open(os.path.join(out, "cube_long3.stl")).read().split("\n")
k = 0
for i, l in enumerate(lines):
    if l.strip().startswith("vertex") and k < 3:
        lines[i] = l + " " * 110; k += 1
open(os.path.join(out, "cube_long3.stl"), "w").write("\n".join(lines))
# every vertex line longer than 99 characters: the count says 17 facets, the read fails
lines = open(os.path.join(out, "cube_long3.stl")).read().split("\n")
lines = [l + " " * 110 if l.strip().startswith("vertex") and len(l) < 99 else l for l in lines]
open(os.path.join(out, "cube_long_all.stl"), "w").write("\n".join(lines))
# a facet with a NaN vertex keeps a zero facet (removed as degenerate)
cn = c[:5] + [((0.0, 0.0, 0.0), (float("nan"), 1.0, 2.0), (3.0, 4.0, 5.0))] + c[5:]
write("cube_nan.stl", cn, fmt="%g")
# trailing text after endloop / endfacet is ignored
write("cube_text.stl", c, fmt="%.9g")
t = open(os.path.join(out, "cube_text.stl")).read().replace("endloop\n", "endloop by writer\n").replace("endfacet\n", "endfacet\tx\n")
open(os.path.join(out, "cube_text.stl"), "w").write(t)
# a facet without "outer loop": the read fails
t = open(os.path.join(out, "cube_text.stl")).read().replace("outer loop", "outer_loop", 1)
open(os.path.join(out, "cube_bad.stl"), "w").write(t)
