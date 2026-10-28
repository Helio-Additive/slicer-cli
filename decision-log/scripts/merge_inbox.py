#!/usr/bin/env python3
"""Move helper inbox entries into the official logs. Each '### title' entry in inbox/<h>.md becomes D-<n>
in DECISIONS.md (accepted by coordinator); porting rows and questions are appended. Merged files move to inbox/merged/."""
import os, re, sys, datetime
d = os.path.dirname(os.path.abspath(__file__)) + "/.."
dec = open(f"{d}/DECISIONS.md").read()
n = max([int(x) for x in re.findall(r"^## D-(\d+)", dec, re.M)] + [99])
n = max(n, 99)
now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
os.makedirs(f"{d}/inbox/merged", exist_ok=True)
for h in sys.argv[1:]:
    p = f"{d}/inbox/{h}.md"
    if os.path.exists(p):
        parts = re.split(r"^### ", open(p).read(), flags=re.M)[1:]
        with open(f"{d}/DECISIONS.md", "a") as o:
            for part in parts:
                n += 1
                title, _, body = part.partition("\n")
                o.write(f"\n## D-{n:03d}  helper \"{h}\", accepted by coordinator {now}\nTitle: {title.strip()}\n{body.rstrip()}\n")
        os.rename(p, f"{d}/inbox/merged/{h}.md")
    for suffix, target in (("-porting", "PORTING-LOG.md"), ("-questions", "QUESTIONS.md")):
        q = f"{d}/inbox/{h}{suffix}.md"
        if os.path.exists(q):
            with open(f"{d}/{target}", "a") as o: o.write(f"\n### From helper {h}\n" + open(q).read().rstrip() + "\n")
            os.rename(q, f"{d}/inbox/merged/{h}{suffix}.md")
print("last decision id", n)
