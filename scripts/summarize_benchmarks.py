#!/usr/bin/env python3
"""Summarize benchmark CSVs into deck-ready numbers. Read-only."""
import csv
import statistics as st
import sys
from collections import defaultdict

SWH = "results/swh_expiry5s.csv"
FA = "amr_fleet_manager/auction_results.csv"


def load(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def reduction(base, new):
    return 100.0 * (base - new) / base if base else 0.0


def stop_and_wait_vs_hybrid(path):
    rows = load(path)
    by_trial = defaultdict(dict)
    for r in rows:
        by_trial[r["trial"]][r["strategy"]] = r
    strategies = sorted({r["strategy"] for r in rows})
    print(f"== {path}: {len(by_trial)} trials")
    for s in strategies:
        rs = [r for r in rows if r["strategy"] == s]
        timeouts = sum(r["timed_out"] == "True" for r in rs)
        collisions = sum(int(r["collisions"]) for r in rs)
        print(f"  {s:14s} completed={len(rs) - timeouts}/{len(rs)}  "
              f"timed_out={timeouts}  collisions={collisions}")
    for n in sorted({int(r["n_robots"]) for r in rows}):
        line = f"  n_robots={n:<3d}"
        for s in strategies:
            rs = [r for r in rows if r["strategy"] == s and int(r["n_robots"]) == n]
            done = sum(r["timed_out"] != "True" for r in rs)
            line += f"  {s}={done}/{len(rs)}"
        print(line)
    rescued = regressed = both_to = 0
    speed = []
    for d in by_trial.values():
        b, h = d.get("stop_and_wait"), d.get("hybrid")
        if not b or not h:
            continue
        bt, ht = b["timed_out"] == "True", h["timed_out"] == "True"
        if bt and ht:
            both_to += 1
        elif bt:
            rescued += 1
        elif ht:
            regressed += 1
        else:
            bm, hm = num(b["makespan_sec"]), num(h["makespan_sec"])
            if bm is not None and hm is not None:
                speed.append(reduction(bm, hm))
    print(f"  baseline timed out but hybrid finished : {rescued}")
    print(f"  hybrid timed out but baseline finished : {regressed}")
    print(f"  both timed out                         : {both_to}")
    if speed:
        print(f"  makespan change where BOTH finished (n={len(speed)}): "
              f"mean {st.mean(speed):.1f}% (positive = hybrid faster)")


def fifo_vs_auction(path):
    rows = load(path)
    by_trial = defaultdict(dict)
    for r in rows:
        by_trial[r["trial"]][r["policy"]] = r
    dist, mk, skipped = [], [], 0
    for d in by_trial.values():
        f, a = d.get("fifo"), d.get("auction")
        vals = [num(x[k]) for x in (f, a) if x for k in ("total_distance", "makespan")]
        if not f or not a or None in vals:
            skipped += 1
            continue
        dist.append(reduction(num(f["total_distance"]), num(a["total_distance"])))
        mk.append(reduction(num(f["makespan"]), num(a["makespan"])))
    print(f"== {path}: {len(dist)} paired trials ({skipped} skipped)")
    for name, xs in (("total distance", dist), ("makespan", mk)):
        if xs:
            print(f"  {name:15s} reduction: mean {st.mean(xs):.1f}%  median {st.median(xs):.1f}%  "
                  f"min {min(xs):.1f}%  max {max(xs):.1f}%  auction better in "
                  f"{sum(x > 0 for x in xs)}/{len(xs)}")


if __name__ == "__main__":
    stop_and_wait_vs_hybrid(sys.argv[1] if len(sys.argv) > 1 else SWH)
    print()
    fifo_vs_auction(sys.argv[2] if len(sys.argv) > 2 else FA)
