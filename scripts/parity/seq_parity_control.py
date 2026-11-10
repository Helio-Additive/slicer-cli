#!/usr/bin/env python3
"""R708: the PERTURBATION CONTROL for `seq_parity.py`'s ordering loss.

Why this exists.

  `seq_parity.py` computes `in_order` with `difflib.get_matching_blocks`, a
  recursive longest-block heuristic, NOT a true LCS. Its own docstring says
  `in_order` is a lower bound -- but a bound is only useful if you know how
  slack it is, and there was reason to fear the slack GROWS with sparsity:
  when matches are scattered singletons the heuristic cannot chain them.

  That mattered because the two fixtures being compared have very different
  match densities. benchy-classic matches 75.1% of lines and reports a 15.27%
  ordering loss; benchy-arachne matches 24.6% and reports 33.98%. If the slack
  scaled with sparsity, the difference between those two numbers would be
  measuring the SPARSITY, not the ordering -- and ten rounds could be spent
  chasing an instrument.

The control follows the rule earned at R518: validate a COMPARATIVE metric by
perturbing one input in a way whose true effect you know exactly.

  Build a synthetic 'rust' from the C++ file by CORRUPTING a fraction p of
  lines IN PLACE. Order is preserved exactly and line count is preserved
  exactly, so the TRUE ordering loss is ZERO by construction at every p.
  Whatever the metric reports is pure instrument artifact.

RESULT (R708, on the 218k-line benchy-arachne C++ reference):

    corrupt%   content%   in-order%   ARTIFACT ordering loss
        0.0%    100.00%     100.00%       0.00%  (0 lines)
       25.0%     74.99%      74.99%       0.00%  (0 lines)   <- classic density
       55.0%     44.95%      44.81%       0.31%  (305 lines)
       75.5%     24.55%      24.25%       1.22%  (652 lines) <- arachne density

  At benchy-classic's density the artifact is ZERO LINES, so classic's 15.27%
  ordering loss is 100% real. At benchy-arachne's density it is 1.22 points of
  the reported 33.98, so ~32.8 points are real. The fear was wrong and the
  ordering loss is a genuine defect worth attacking. Re-run this whenever the
  ordering figure is used to compare two fixtures of different densities.

Usage: seq_parity_control.py <cpp.gcode> [p ...]
"""
import random
import sys
from collections import Counter
from difflib import SequenceMatcher

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from line_parity import DEC, groups, quantise  # noqa: E402

SEED = 20260810


def corrupt(line, rng):
    """Make a line that cannot quantise-match, without changing its position.

    Shifting one coordinate by 1 mm is far outside the 1e-3 tolerance and keeps
    the line's shape, so the (layer, feature) grouping is unaffected.
    """
    out, changed = [], False
    for tok in line.split(' '):
        if not changed and len(tok) > 1 and tok[0] in 'XYZEF':
            try:
                out.append(f"{tok[0]}{float(tok[1:]) + 1.0:.5f}")
                changed = True
                continue
            except ValueError:
                pass
        out.append(tok)
    if not changed:
        return line + f" ;ctl-{rng.randrange(1 << 30)}"
    return ' '.join(out)


def score(rg, cg, dec=DEC):
    """seq_parity's two counters, recomputed on the synthetic pair."""
    tot_r = sum(len(v) for v in rg.values())
    matched = in_order = 0
    for key in set(rg) | set(cg):
        r = [quantise(x, dec) for x in rg.get(key, [])]
        c = [quantise(x, dec) for x in cg.get(key, [])]
        matched += sum((Counter(r) & Counter(c)).values())
        in_order += sum(b.size for b in
                        SequenceMatcher(None, r, c, autojunk=False).get_matching_blocks())
    return tot_r, matched, in_order


def main():
    cp = sys.argv[1]
    ps = [float(x) for x in sys.argv[2:]] or [0.0, 0.10, 0.25, 0.40, 0.55, 0.70, 0.755]
    cg = groups(cp)

    print(f"control file: {cp}")
    print("perturbation is IN PLACE -- order and line count identical, so the")
    print("TRUE ordering loss is 0 for every row. Anything reported is artifact.\n")
    print(f"{'corrupt%':>9} {'content%':>9} {'in-order%':>10} "
          f"{'ARTIFACT ordering loss':>24}")
    for p in ps:
        rng = random.Random(SEED)
        rg = {k: [corrupt(l, rng) if rng.random() < p else l for l in v]
              for k, v in cg.items()}
        tot, m, o = score(rg, cg)
        loss = 100 * (m - o) / max(m, 1)
        print(f"{100*p:>8.1f}% {100*m/tot:>8.2f}% {100*o/tot:>9.2f}% "
              f"{loss:>21.2f}%  ({m-o} lines)")


if __name__ == '__main__':
    main()
