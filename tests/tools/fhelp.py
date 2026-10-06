#!/usr/bin/env python3
"""fhelp.py: small readers for findings.sh (G-code, STL, 3MF, result.json).

Usage: fhelp.py <command> args...
  stl_boxes DIR|FILE...     JSON list of [minx,maxx,miny,maxy,minz,maxz] per STL (sorted, 1 decimal)
  labels GCODE              JSON list of the 'unique label id' values, in first-print order
  label_count GCODE         number of distinct label ids printed
  hdr GCODE KEY             the config-block value of KEY ('' when absent)
  items 3MF                 JSON list of build items: objectid, printable, identify_id, size, min
  info_minx STDOUT          JSON list of the min_x values --info printed
  json FILE KEY             a top-level JSON value
"""
import json, os, re, struct, sys, zipfile


def read_stl(path):
    data = open(path, "rb").read()
    pts = []
    if data[:5] == b"solid" and b"facet" in data[:400]:
        for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data):
            pts.append(tuple(float(x) for x in m.groups()))
    else:
        n = struct.unpack_from("<I", data, 80)[0]
        for i in range(n):
            v = struct.unpack_from("<12f", data, 84 + 50 * i)
            pts += [v[3:6], v[6:9], v[9:12]]
    return pts


def box(pts):
    return [round(f(p[i] for p in pts), 1) for i in range(3) for f in (min, max)]


def stl_boxes(args):
    files = []
    for a in args:
        if os.path.isdir(a):
            files += [os.path.join(a, n) for n in sorted(os.listdir(a)) if n.lower().endswith(".stl")]
        elif os.path.exists(a):
            files.append(a)
    print(json.dumps(sorted(box(read_stl(f)) for f in files)))


def labels(path):
    seen, out = set(), []
    with open(path, errors="replace") as f:
        for line in f:
            if "start printing object, unique label id:" in line:
                v = int(line.rsplit(":", 1)[1].strip().split()[0])
                if v not in seen:
                    seen.add(v); out.append(v)
    return out


def hdr(path, key):
    pat = re.compile(r"^; " + re.escape(key) + r" = (.*)$")
    with open(path, errors="replace") as f:
        for line in f:
            m = pat.match(line.rstrip("\r\n"))
            if m:
                return m.group(1)
    return ""


def items(path):
    z = zipfile.ZipFile(path)
    files = {n: z.read(n).decode("utf-8", "replace") for n in z.namelist() if n.endswith(".model")}
    mat = lambda t: [float(x) for x in t.split()] if t else [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0]
    app = lambda m, p: tuple(p[0] * m[i] + p[1] * m[3 + i] + p[2] * m[6 + i] + m[9 + i] for i in range(3))

    def mul(a, b):
        return [sum(a[3 * r + k] * b[3 * k + c] for k in range(3)) for r in range(3) for c in range(3)] + list(app(b, a[9:12]))

    attr = lambda s, k: (re.search(r"(?:^|\s)" + k + r"=\"([^\"]+)\"", s) or [None, None])[1]

    def obj(fname, oid):
        body = re.search(r"<object id=\"%s\"[^>]*>(.*?)</object>" % oid, files[fname.lstrip("/")], re.S).group(1)
        pts = [tuple(map(float, v)) for v in re.findall(r"<vertex x=\"([^\"]+)\" y=\"([^\"]+)\" z=\"([^\"]+)\"", body)]
        return pts, re.findall(r"<component ([^>]*)/>", body)

    # identify_id per (object_id, instance_id) from the plates in model_settings.config.
    ident = {}
    plate_of = {}
    try:
        ms = z.read("Metadata/model_settings.config").decode("utf-8", "replace")
        for pi, plate in enumerate(re.findall(r"<plate>(.*?)</plate>", ms, re.S)):
            pid = re.search(r"key=\"plater_id\" value=\"(\d+)\"", plate)
            pid = int(pid.group(1)) if pid else pi + 1
            for mi in re.findall(r"<model_instance>(.*?)</model_instance>", plate, re.S):
                g = lambda k: (re.search(r"key=\"%s\" value=\"(-?\d+)\"" % k, mi) or [None, None])[1]
                key = (g("object_id"), g("instance_id"))
                ident[key] = g("identify_id")
                plate_of[key] = pid
    except KeyError:
        pass
    root = "3D/3dmodel.model"
    build = files[root][files[root].index("<build"):]
    out, per_obj = [], {}
    for it in re.findall(r"<item ([^>]*)/>", build):
        oid = attr(it, "objectid")
        k = per_obj.get(oid, 0); per_obj[oid] = k + 1
        tm = mat(attr(it, "transform"))
        pts, comps = obj(root, oid)
        world = [app(tm, p) for p in pts]
        for c in comps:
            cp, _ = obj(attr(c, "p:path") or root, attr(c, "objectid"))
            world += [app(mul(mat(attr(c, "transform")), tm), p) for p in cp]
        lo = [min(p[i] for p in world) for i in range(3)]
        hi = [max(p[i] for p in world) for i in range(3)]
        out.append({"objectid": oid, "instance": k, "printable": attr(it, "printable") not in ("0", "false"),
                    "identify_id": ident.get((oid, str(k))), "plate": plate_of.get((oid, str(k))),
                    "size": [round(hi[i] - lo[i]) for i in range(3)], "min": [round(lo[i], 1) for i in range(3)]})
    return out


def main():
    cmd, args = sys.argv[1], sys.argv[2:]
    if cmd == "stl_boxes":
        stl_boxes(args)
    elif cmd == "labels":
        print(json.dumps(labels(args[0])))
    elif cmd == "label_count":
        print(len(labels(args[0])))
    elif cmd == "hdr":
        print(hdr(args[0], args[1]))
    elif cmd == "items":
        print(json.dumps(items(args[0])))
    elif cmd == "info_minx":
        t = open(args[0], errors="replace").read()
        print(json.dumps([round(float(v), 1) for v in re.findall(r"^min_x = (\S+)", t, re.M)]))
    elif cmd == "json":
        d = json.load(open(args[0]))
        for k in args[1].split("."):
            d = d[int(k)] if isinstance(d, list) else d[k]
        print(json.dumps(d))
    else:
        sys.exit("unknown command " + cmd)


if __name__ == "__main__":
    main()
