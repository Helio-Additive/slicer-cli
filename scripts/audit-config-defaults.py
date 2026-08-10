#!/usr/bin/env python3
"""Audit our config struct defaults against C++'s `set_default_value` (R707).

WHY THIS EXISTS
---------------
R706 found three arachne limits 250-850x too small. The cause was not logic and
not parsing: `print_config.rs` simply defaulted `min_bead_width` /
`min_feature_size` / `wall_transition_length` to millimetre-ish guesses where
C++'s `PrintConfig.cpp` declares them `coPercent` with defaults 85 / 25 / 100.
Nothing caught it because NO profile in the inheritance chain declares those
keys, so both engines fell back to their own defaults and only ours was wrong.

That bug class is mechanical, so this makes it repeatable:
  C++   `def = this->add("KEY", coTYPE); ... def->set_default_value(new ...(V));`
  Rust  key -> field via the `set_deserialize` match arms, field -> default via
        the struct-literal `field: value,` lines.

A MISMATCH IS NOT AUTOMATICALLY A BUG
-------------------------------------
A default only bites when NO config declares the key. Before chasing one, check
whether the fixtures declare it (benchy's profile inheritance chain, Majora's
Metadata/project_settings.config). R707 swept 24 mismatches and found ALL of
them inert: 16 declared by both fixtures; the raft/ironing ones unreachable
(`raft_layers = 0`, `ironing_type = "no ironing"`); the jerk ones verified
identical in emitted M205; and `elefant_foot_min_width` consumed only by SLA.

COVERAGE IS PARTIAL — DO NOT READ "0 LIVE MISMATCHES" AS "NONE EXIST"
--------------------------------------------------------------------
The key->field mapping is heuristic (regex over match arms) and only NUMERIC
struct-literal defaults are compared. Enum, bool, string and vector options are
out of scope, as are fields whose default is computed rather than literal. The
counts printed below say how much was actually compared.

Usage:  python3 scripts/audit-config-defaults.py
"""
import re, json
CPP='libslic3r/bambustudio/references/BambuStudio/src/libslic3r/PrintConfig.cpp'
RS ='crates/libslic3r-rs/src/print_config.rs'

# --- C++: key -> (type, default)
src=open(CPP,errors='replace').read()
cpp={}
# each option block starts at `def = this->add("KEY", coTYPE);`
blocks=re.split(r'\n\s*def\s*=\s*this->add\(', src)
for b in blocks[1:]:
    m=re.match(r'"([^"]+)"\s*,\s*(co\w+)', b)
    if not m: continue
    key, ctype = m.group(1), m.group(2)
    # first set_default_value in this block
    d=re.search(r'set_default_value\(new ConfigOption\w*\(([^;]*?)\)\s*\)\s*;', b)
    if not d: continue
    val=d.group(1).strip()
    cpp[key]=(ctype, val)
print(f"C++ options with an explicit default: {len(cpp)}")

# --- Rust: key -> field, from set_deserialize / apply_key_value match arms
rs=open(RS,errors='replace').read()
key2field={}
for m in re.finditer(r'"([a-z0-9_]+)"\s*=>\s*\{(.{0,400}?)\n\s{8,12}\}', rs, re.S):
    key, body = m.group(1), m.group(2)
    f=re.search(r'self\.([a-z0-9_]+)\s*=', body)
    if f: key2field.setdefault(key, f.group(1))
print(f"Rust keys mapped to fields:            {len(key2field)}")

# --- Rust: field -> default, from struct-literal `field: value,` lines
field2def={}
for m in re.finditer(r'^\s{8,16}([a-z0-9_]+):\s*([0-9][0-9_]*\.?[0-9]*(?:e-?\d+)?)\s*,\s*$', rs, re.M):
    field2def.setdefault(m.group(1), m.group(2))
print(f"Rust numeric struct defaults found:    {len(field2def)}")
print()

def num(s):
    s=s.strip().rstrip('f').replace('_','')
    try: return float(s)
    except: return None

rows=[]
for key,(ctype,val) in sorted(cpp.items()):
    if key not in key2field: continue
    field=key2field[key]
    if field not in field2def: continue
    c=num(val); r=num(field2def[field])
    if c is None or r is None: continue
    if abs(c-r) < 1e-12: continue
    ratio = (c/r) if r not in (0.0,) else float('inf')
    rows.append((abs(ratio if ratio>=1 else 1/ratio) if ratio not in (0.0,) else 0, key, ctype, c, r, ratio, field))
rows.sort(reverse=True)
print(f"{'key':38}{'ctype':12}{'C++':>12}{'ours':>12}{'C++/ours':>12}")
for _,key,ctype,c,r,ratio,field in rows[:40]:
    print(f"  {key:36}{ctype:12}{c:12g}{r:12g}{ratio:12.4g}   ({field})")
print(f"\nTOTAL MISMATCHES: {len(rows)}")
