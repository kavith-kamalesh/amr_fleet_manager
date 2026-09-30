#!/usr/bin/env python3
"""How robust is the timeout-penalized improvement claim? Read-only."""
import csv
import random
import sys
from collections import defaultdict

BASE, NEW = "stop_and_wait", "hybrid"
DEFAULT = "results/swh_expiry5s.csv"
CAP = 60.0


def load(path):
    trials = defaultdict(dict)
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            t = float(r["makespan_sec"]) if r["makespan_sec"] else None
            timed_out = r["timed_out"] == "True" or t is None
            trials[int(r["trial"])][r["strategy"]] = (timed_out, t, int(r["n_robots"]))
    return {k: v for k, v in trials.items() if BASE in v and NEW in v}


def mean_time(ids, trials, strat, penalty):
    vals = []
    for k in ids:
        timed_out, t, _ = trials[k][strat]
        vals.append(penalty if timed_out else t)
    return sum(vals) / len(vals)


def improvement(ids, trials, penalty):
    b = mean_time(ids, trials, BASE, penalty)
    n = mean_time(ids, trials, NEW, penalty)
    return 100.0 * (b - n) / b


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT
    trials = load(path)
    ids = sorted(trials)
    to_b = sum(trials[k][BASE][0] for k in ids)
    to_n = sum(trials[k][NEW][0] for k in ids)
    print(f"{len(ids)} paired trials | timeouts: {BASE}={to_b}, {NEW}={to_n}")

    print("\nImprovement vs timeout penalty (PS target >= 20%):")
    for p in (CAP, 90.0, 120.0, 300.0, 600.0):
        print(f"  penalty {p:>5.0f}s : {improvement(ids, trials, p):+6.1f}%")
    if to_b:
        print(f"  penalty -> inf  : {100.0 * (1 - to_n / to_b):+6.1f}%  "
              f"(limit = 1 - timeouts_{NEW} / timeouts_{BASE})")

    rng = random.Random(42)
    draws = sorted(
        improvement([rng.choice(ids) for _ in ids], trials, CAP)
        for _ in range(2000)
    )
    lo = draws[int(0.025 * len(draws))]
    hi = draws[int(0.975 * len(draws)) - 1]
    share = 100.0 * sum(d >= 20.0 for d in draws) / len(draws)
    print(f"\nBootstrap (2000 resamples, seed 42) at {CAP:.0f}s penalty:")
    print(f"  95% CI [{lo:+.1f}%, {hi:+.1f}%]; >= 20% in {share:.1f}% of resamples")

    print(f"\nBy fleet size at {CAP:.0f}s penalty:")
    for n in sorted({trials[k][BASE][2] for k in ids}):
        sub = [k for k in ids if trials[k][BASE][2] == n]
        tb = sum(trials[k][BASE][0] for k in sub)
        tn = sum(trials[k][NEW][0] for k in sub)
        print(f"  n_robots={n}: {improvement(sub, trials, CAP):+6.1f}%  "
              f"(trials={len(sub)}, timeouts {BASE}={tb}, {NEW}={tn})")


if __name__ == "__main__":
    main()
