#!/usr/bin/env python3
"""Replay hybrid timeouts from the committed results with a read-only tracer.

Classifies each timeout as deadlock-like (nobody moves) or livelock candidate
(robots keep moving and rerouting but never arrive). The tracer only records
state, and every replay is checked against the committed CSV row before its
result is trusted.
"""
import argparse
import csv
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "amr_fleet_manager"))
import benchmark_stop_and_wait_vs_hybrid as bench  # noqa: E402

WINDOW_SEC = 20.0
MOVE_THRESH_M = 1.0
NEAR_M = 1.4


def load_expected(path):
    with open(path, newline="") as f:
        return {(int(r["trial"]), r["strategy"]): r for r in csv.DictReader(f)}


def configs_up_to(max_trial, seed):
    rng = np.random.default_rng(seed)
    configs = {}
    for i in range(max_trial + 1):
        n = int(rng.integers(3, 7))
        configs[i] = bench.random_scenario(rng, n)
    return configs


def traced_run(config, strategy):
    snaps = defaultdict(dict)
    robots = []
    real_cls = bench.SimRobot

    class TracedRobot(real_cls):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            robots.append(self)

        def edge_time_window(self, t_now):
            key = round(t_now, 3)
            if key not in snaps[self.id]:
                snaps[self.id][key] = (
                    self.pos.copy(),
                    tuple(self.path) if self.path else None,
                    self.wait_start_time,
                )
            return super().edge_time_window(t_now)

    bench.SimRobot = TracedRobot
    try:
        result = bench.run_scenario(config, strategy)
    finally:
        bench.SimRobot = real_cls
    return result, robots, snaps


def analyse(robots, snaps):
    t_end = max((t for s in snaps.values() for t in s), default=0.0)
    rows = []
    for r in robots:
        s = snaps.get(r.id, {})
        times = [t for t in sorted(s) if t >= t_end - WINDOW_SEC]
        row = {"id": r.id, "arrived": r.arrived, "dist": 0.0, "net": 0.0,
               "reroutes": 0, "wait_now": 0.0, "pos": None, "path": None}
        if not r.arrived and len(times) >= 2:
            pairs = list(zip(times, times[1:]))
            row["dist"] = float(sum(np.linalg.norm(s[b][0] - s[a][0]) for a, b in pairs))
            row["net"] = float(np.linalg.norm(s[times[-1]][0] - s[times[0]][0]))
            row["reroutes"] = sum(1 for a, b in pairs if s[a][1] != s[b][1])
            last_wait = s[times[-1]][2]
            row["wait_now"] = t_end - last_wait if last_wait is not None else 0.0
            row["pos"] = s[times[-1]][0]
            row["path"] = s[times[-1]][1]
        rows.append(row)
    return rows


def verdict(rows):
    open_rows = [x for x in rows if not x["arrived"]]
    if not open_rows:
        return "replay shows everyone arrived (unexpected)"
    moving = [x for x in open_rows if x["dist"] > MOVE_THRESH_M]
    n_rr = sum(x["reroutes"] for x in open_rows)
    if not moving:
        return f"DEADLOCK-like: none of {len(open_rows)} unfinished robots moved > {MOVE_THRESH_M:.0f} m in the last {WINDOW_SEC:.0f} s"
    if n_rr >= 3:
        return f"LIVELOCK candidate: {len(moving)}/{len(open_rows)} unfinished moving, {n_rr} reroutes in last {WINDOW_SEC:.0f} s, none arriving"
    return f"SLOW/BLOCKED: {len(moving)}/{len(open_rows)} unfinished moving, only {n_rr} reroutes"


def print_detail(trial, config, rows):
    print(f"--- trial {trial} detail (last {WINDOW_SEC:.0f} s) ---")
    open_pos = {x["id"]: x["pos"] for x in rows if x["pos"] is not None}
    for x in rows:
        start, goal, prio = config[x["id"]]
        head = f"  robot {x['id']}: {start}->{goal} prio={prio:.2f}"
        if x["arrived"]:
            print(head + "  ARRIVED")
            continue
        near = [i for i, p in open_pos.items()
                if i != x["id"] and x["pos"] is not None and np.linalg.norm(p - x["pos"]) < NEAR_M]
        pos = tuple(round(float(v), 1) for v in x["pos"]) if x["pos"] is not None else None
        print(f"{head}  moved={x['dist']:.1f}m net={x['net']:.1f}m reroutes={x['reroutes']} "
              f"wait_now={x['wait_now']:.1f}s pos={pos} near={near}")
        if x["path"]:
            print(f"           current path: {list(x['path'])}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="results/swh_expiry5s.csv")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--detail", type=int, nargs="*", default=[])
    a = ap.parse_args()

    exp = load_expected(a.csv)
    timeouts = sorted(t for (t, s), r in exp.items() if s == "hybrid" and r["timed_out"] == "True")
    configs = configs_up_to(max(timeouts), a.seed)
    print(f"{len(timeouts)} hybrid timeouts to replay\n")
    print("trial  n  open  verdict")
    for t in timeouts:
        result, robots, snaps = traced_run(configs[t], "hybrid")
        row = exp[(t, "hybrid")]
        if result["timed_out"] != (row["timed_out"] == "True") or \
                result["total_reroutes"] != int(row["total_reroutes"]):
            sys.exit(f"REPLAY MISMATCH on trial {t}: got timed_out={result['timed_out']} "
                     f"reroutes={result['total_reroutes']}, CSV has {row['timed_out']} "
                     f"reroutes={row['total_reroutes']}. The scenario generator differs from main().")
        rows = analyse(robots, snaps)
        n_open = sum(1 for x in rows if not x["arrived"])
        print(f"{t:5d} {len(robots):2d} {n_open:5d}  {verdict(rows)}")
        if t in a.detail:
            print_detail(t, configs[t], rows)


if __name__ == "__main__":
    main()
