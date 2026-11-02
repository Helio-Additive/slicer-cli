#!/usr/bin/env python3
"""check_json_schema.py: check JSON documents against one of schemas/.

Usage: check_json_schema.py SCHEMA DOC...
       check_json_schema.py --events SCHEMA FILE...   (each "[[SLICER_EVENT]] <JSON>" line of each file)

Standard library only, so it runs on every CI runner. It knows the JSON Schema
keywords schemas/ use and stops on any other anywhere in the schema, before it
reads a document, so a schema keyword it does not check can never pass
silently. A document with NaN or Infinity (not JSON numbers), an event line
that is not UTF-8 and an event marker without its space fail.
"""
import json, sys

ANNOTATIONS = {"$schema", "$id", "$defs", "title", "description"}
TYPES = {"object": dict, "array": list, "string": str, "boolean": bool, "null": type(None)}


def is_type(v, t):
    if t in ("integer", "number"):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return False
        # Draft 2020-12: a number with no fractional part is an integer (1.0).
        return t == "number" or isinstance(v, int) or v.is_integer()
    return isinstance(v, TYPES[t])


def walk(s, at):
    """Stops on a keyword this checker does not implement, at any depth."""
    if not isinstance(s, dict):
        return
    for k in s:
        if k not in ANNOTATIONS and k not in CHECKS:
            raise SystemExit(f"check_json_schema.py: unsupported keyword {k!r} at {at}")
    for k in ("properties", "$defs"):
        for name, sub in s.get(k, {}).items():
            walk(sub, f"{at}/{k}/{name}")
    for k in ("items", "additionalProperties"):
        walk(s.get(k), f"{at}/{k}")
    for i, sub in enumerate(s.get("oneOf", [])):
        walk(sub, f"{at}/oneOf/{i}")


def errors(v, s, root, at):
    """The reasons v does not match schema s, as 'path: reason' strings."""
    out = []
    for k, check in CHECKS.items():
        if k in s:
            out += check(v, s[k], s, root, at)
    return out


def ref(v, r, s, root, at):
    assert r.startswith("#/"), r
    node = root
    for part in r[2:].split("/"):
        node = node[part]
    return errors(v, node, root, at)


def one_of(v, subs, s, root, at):
    found = [errors(v, sub, root, at) for sub in subs]
    n = sum(not f for f in found)
    if n == 1:
        return []
    # None matched: each branch's reasons, so the failure says what is wrong.
    return [f"{at}: matches {n} of the oneOf schemas, not 1"] + [e for f in found for e in f]


def type_(v, t, s, root, at):
    ts = t if isinstance(t, list) else [t]
    return [] if any(is_type(v, x) for x in ts) else [f"{at}: {json.dumps(v)} is not {t}"]


def properties(v, props, s, root, at):
    if not isinstance(v, dict):
        return []
    out = []
    for k, sub in props.items():
        if k in v:
            out += errors(v[k], sub, root, f"{at}.{k}")
    if s.get("additionalProperties", True) is False:
        out += [f"{at}: unexpected field {k!r}" for k in v if k not in props]
    return out


def additional(v, a, s, root, at):
    # false is checked with properties; a schema is every field properties does not name.
    if a is True or a is False or not isinstance(v, dict):
        return []
    named = s.get("properties", {})
    return [e for k, x in v.items() if k not in named for e in errors(x, a, root, f"{at}.{k}")]


def required(v, keys, s, root, at):
    return [f"{at}: missing field {k!r}" for k in keys if isinstance(v, dict) and k not in v]


def items(v, sub, s, root, at):
    return [e for i, x in enumerate(v) for e in errors(x, sub, root, f"{at}[{i}]")] if isinstance(v, list) else []


CHECKS = {
    "$ref": ref,
    "oneOf": one_of,
    "type": type_,
    "const": lambda v, c, *_: [] if v == c and type(v) is type(c) else [f"{_[-1]}: {json.dumps(v)} is not {json.dumps(c)}"],
    "enum": lambda v, e, *_: [] if v in e else [f"{_[-1]}: {json.dumps(v)} is not one of {e}"],
    "properties": properties,
    "additionalProperties": additional,
    "required": required,
    "items": items,
    "minItems": lambda v, n, *_: [f"{_[-1]}: fewer than {n} items"] if isinstance(v, list) and len(v) < n else [],
    "maxItems": lambda v, n, *_: [f"{_[-1]}: more than {n} items"] if isinstance(v, list) and len(v) > n else [],
    "minimum": lambda v, n, *_: [f"{_[-1]}: {v} is below {n}"] if is_type(v, "number") and v < n else [],
    "maximum": lambda v, n, *_: [f"{_[-1]}: {v} is above {n}"] if is_type(v, "number") and v > n else [],
}


MARKER = "[[SLICER_EVENT]]"


def no_constant(name):
    raise ValueError(f"{name} is not a JSON number")


def parse(text):
    """(document, None), or (None, reason) when text is not strict JSON."""
    try:
        return json.loads(text, parse_constant=no_constant), None
    except ValueError as e:
        return None, f"not JSON: {e}"


def documents(paths, events):
    """(name, document, reason) for each file, or for each event line of each file."""
    for path in paths:
        if not events:
            yield (path,) + parse(open(path, encoding="utf-8").read())
            continue
        # Bytes, so only an event line is decoded, and strictly: an invalid
        # UTF-8 byte in an event fails, while other output lines are not read.
        marker = MARKER.encode()
        for n, line in enumerate(open(path, "rb"), 1):
            if line.startswith(marker + b" "):
                try:
                    yield (f"{path}:{n}",) + parse(line[len(marker) + 1:].decode("utf-8"))
                except UnicodeDecodeError as e:
                    yield f"{path}:{n}", None, f"not UTF-8: {e}"
            elif line.startswith(marker):
                yield f"{path}:{n}", None, f"no space after {MARKER}"


def main():
    args = sys.argv[1:]
    events = bool(args) and args[0] == "--events"
    if events:
        args = args[1:]
    if len(args) < 2:
        raise SystemExit(__doc__)
    schema = json.load(open(args[0]))
    walk(schema, "#")
    total = bad = 0
    for name, doc, reason in documents(args[1:], events):
        found = [f"$: {reason}"] if reason else errors(doc, schema, schema, "$")
        for e in found:
            print(f"{name}: {e}")
        total += 1
        bad += bool(found)
    print(f"{total - bad} of {total} documents match {args[0]}")
    sys.exit(1 if bad or not total else 0)


if __name__ == "__main__":
    main()
