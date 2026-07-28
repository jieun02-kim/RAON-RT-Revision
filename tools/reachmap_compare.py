#!/usr/bin/env python3
"""[kv260-merge] Compare two reach maps cell by cell.

    python3 tools/reachmap_compare.py baseline.csv candidate.csv

Written for the ik-multistart branch, where the acceptance criteria were fixed
BEFORE the code was: a solver change is only an improvement if it can be shown
to be one against a map produced by the solver it replaces.

The report answers, in order:

  1. LOST cells    — accepted by the baseline, refused by the candidate.
                     For multi-start this must be EMPTY by construction (seed 0
                     is q_ready, so the baseline's own solution is always in the
                     candidate set). A non-empty list is a bug in the ranking,
                     not a trade-off.
  2. GAINED cells  — refused by the baseline, accepted by the candidate. This
                     is the whole point.
  3. Timing        — mean / p99 / max. The 1 kHz budget is 1.00 ms and the
                     baseline's worst was 56 ms; a candidate that fixes reach
                     while making the worst case worse has not helped.
  4. Verdict flow  — which refusal reasons turned into what.

Both files must be the same grid AND the same anchor: pass --seed explicitly to
reachmap.out on both runs, or you are comparing anchors, not solvers.
"""

import csv
import sys
from collections import Counter

import numpy as np


def read(path):
    rows, q_ready = {}, None
    with open(path, newline="") as fh:
        header = None
        for line in fh:
            if line.startswith("#"):
                if line[1:].strip().startswith("q_ready"):
                    q_ready = line[1:].strip()
                continue
            if header is None:
                header = next(csv.reader([line]))
                continue
            r = dict(zip(header, next(csv.reader([line]))))
            key = (round(float(r["x"]), 4), round(float(r["y"]), 4),
                   round(float(r["z"]), 4))
            rows[key] = r
    if not rows:
        sys.exit(f"{path}: no data rows")
    return rows, q_ready


def stats(rows, keys, field="ms"):
    v = np.array([float(rows[k][field]) for k in keys])
    return v.mean(), np.percentile(v, 99), v.max()


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    base, base_q = read(sys.argv[1])
    cand, cand_q = read(sys.argv[2])

    if base_q != cand_q:
        print("!! DIFFERENT ANCHORS — this compares anchors, not solvers")
        print(f"   baseline : {base_q}")
        print(f"   candidate: {cand_q}\n")

    shared = sorted(set(base) & set(cand))
    if not shared:
        sys.exit("no overlapping cells — different grids?")
    only_b, only_c = len(base) - len(shared), len(cand) - len(shared)
    if only_b or only_c:
        print(f"!! grids differ: {only_b} baseline-only, {only_c} "
              f"candidate-only cells ignored\n")

    bp = {k for k in shared if base[k]["verdict"] == "PASS"}
    cp = {k for k in shared if cand[k]["verdict"] == "PASS"}
    lost, gained = sorted(bp - cp), sorted(cp - bp)

    print(f"cells compared      : {len(shared)}")
    print(f"baseline  PASS      : {len(bp)}  ({100*len(bp)/len(shared):.1f}%)")
    print(f"candidate PASS      : {len(cp)}  ({100*len(cp)/len(shared):.1f}%)")
    print(f"  gained            : +{len(gained)}")
    print(f"  LOST              : -{len(lost)}"
          f"{'   <-- must be 0' if lost else ''}")

    if lost:
        print("\n  first lost cells (x, y, z -> candidate verdict):")
        for k in lost[:10]:
            print(f"    {k[0]:+.3f} {k[1]:+.3f} {k[2]:.3f} -> "
                  f"{cand[k]['verdict']} (gap {cand[k]['gap_rad']}, "
                  f"tilt {cand[k]['tilt_deg']})")

    print("\ntiming over all shared cells [ms]")
    for name, rows in (("baseline ", base), ("candidate", cand)):
        m, p99, mx = stats(rows, shared)
        print(f"  {name}: mean {m:6.2f}   p99 {p99:7.2f}   max {mx:7.2f}")

    flow = Counter((base[k]["verdict"], cand[k]["verdict"]) for k in shared
                   if base[k]["verdict"] != cand[k]["verdict"])
    if flow:
        print("\nverdict changes (baseline -> candidate)")
        for (b, c), n in flow.most_common():
            print(f"  {b:12s} -> {c:12s} : {n}")
    else:
        print("\nno verdict changed")

    if "seed" in next(iter(cand.values())):
        seeds = Counter(cand[k]["seed"] for k in cp)
        print("\nwinning seed among candidate PASS cells")
        for s, n in sorted(seeds.items(), key=lambda kv: -kv[1]):
            print(f"  seed {s:>2s} : {n}")

    print("\nverdict: " + ("FAIL — cells were lost" if lost else
                           "no regression"))


if __name__ == "__main__":
    main()
