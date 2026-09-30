#!/usr/bin/env python3
"""Completion rates and timeout-penalized improvement, split by whether a trial is solvable
by construction.

With PARKED_BLOCKS an arrived robot stays on the grid, so if two robots share a goal node the
second can never arrive. Whether a trial has a shared goal is fixed by the scenario generator
before anything runs, so this split does not depend on outcomes (unlike dropping trials that
one strategy failed).
"""
import csv
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "amr_fleet_manager"))
import trace_hybrid_timeouts as t  # noqa: E402

CAP = 60.0
SEED = 42


def load(path):
    trials = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            mk = float(r["makespan_sec"]) if r["makespan_sec"] else None
            trials.setdefault(int(r["trial"]), {})[r["strategy"]] = (r["timed_out"] == "True", mk)
    return {k: v for k, v in trials.items() if "hybrid" in v and "stop_and_wait" in v}


def stats(trials, ids):
    out = {}
    for strat in ("stop_and_wait", "hybrid"):
        done, vals = 0, []
        for k in ids:
            timed_out, mk = trials[k][strat]
            done += 0 if timed_out else 1
            vals.append(CAP if timed_out else mk)
        out[strat] = (done, sum(vals) / len(vals) if vals else float("nan"))
    return out


def improvement(s):
    b, h = s["stop_and_wait"][1], s["hybrid"][1]
    return 100.0 * (b - h) / b if b else float("nan")


def report(path):
    trials = load(path)
    cfg = t.configs_up_to(max(trials), SEED)
    shared = {k for k in trials if len({g for _, g, _ in cfg[k]}) < len(cfg[k])}
    print(f"== {path}")
    for label, ids in (("all trials", sorted(trials)),
                       ("solvable (no shared goal)", sorted(set(trials) - shared)),
                       ("shared goal", sorted(shared))):
        if not ids:
            continue
        s = stats(trials, ids)
        n = len(ids)
        print(f"  {label:27s} n={n:3d}  hybrid {s['hybrid'][0]:3d}/{n}  stop_and_wait {s['stop_and_wait'][0]:3d}/{n}"
              f"  time saved @ {CAP:.0f}s cap: {improvement(s):+6.1f}%")


if __name__ == "__main__":
    for p in (sys.argv[1:] or sorted(glob.glob("results/swh_*.csv"))):
        report(p)
        print()
