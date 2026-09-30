#!/usr/bin/env python3
"""Odometry recorder + separation analysis for the ROS smoke test.

  record  : subscribe to /robotN/odom for --duration seconds, then exit by itself.
            Writes CSV (t, robot, x, y) in the world frame (spawn offsets applied).
  analyze : pure Python, no ROS needed. Minimum pairwise separation, time spent
            below the collision distance, and path length travelled per robot.
"""
import argparse
import csv
import math
import time
from collections import defaultdict

OFFSETS = {"robot1": (0.0, 0.0), "robot2": (4.0, 0.0), "robot3": (0.0, 4.0)}


def analyze_rows(rows, threshold):
    last, travelled = {}, defaultdict(float)
    min_d, min_at = math.inf, None
    breach_s, first, lastb = 0.0, None, None
    prev_t, prev_in = None, False
    for t, name, x, y in sorted(rows, key=lambda r: r[0]):
        if prev_t is not None and prev_in:
            breach_s += t - prev_t
        if name in last:
            travelled[name] += math.dist(last[name], (x, y))
        last[name] = (x, y)
        in_breach = False
        for other, pos in last.items():
            if other == name:
                continue
            d = math.dist((x, y), pos)
            if d < min_d:
                min_d, min_at = d, (t,) + tuple(sorted((name, other)))
            if d < threshold:
                in_breach = True
        if in_breach:
            first = t if first is None else first
            lastb = t
        prev_t, prev_in = t, in_breach
    return {"robots": sorted(last), "min_d": min_d, "min_at": min_at, "breach_s": breach_s,
            "first_breach": first, "last_breach": lastb, "final": dict(last),
            "travelled": dict(travelled), "samples": len(rows)}


def pair_breaches(rows, threshold, step=0.1):
    """Per-pair seconds below `threshold` and minimum distance, sampled on a fixed time grid.
    Returns (pair_seconds, pair_min_distance, seconds_any_pair_below)."""
    import itertools
    rows = sorted(rows, key=lambda r: r[0])
    last, seconds, min_d, any_s, idx = {}, defaultdict(float), {}, 0.0, 0
    t, t_end = rows[0][0], rows[-1][0]
    while t <= t_end:
        while idx < len(rows) and rows[idx][0] <= t:
            _, n, x, y = rows[idx]
            last[n] = (x, y)
            idx += 1
        hit = False
        for a, b in itertools.combinations(sorted(last), 2):
            d = math.dist(last[a], last[b])
            min_d[(a, b)] = min(min_d.get((a, b), math.inf), d)
            if d < threshold:
                seconds[(a, b)] += step
                hit = True
        if hit:
            any_s += step
        t += step
    return dict(seconds), min_d, any_s


def record(args):
    import rclpy
    from nav_msgs.msg import Odometry
    rclpy.init()
    node = rclpy.create_node("smoke_odom_recorder")
    rows, t0 = [], time.monotonic()

    def make_cb(name):
        ox, oy = OFFSETS[name]

        def cb(msg):
            p = msg.pose.pose.position
            rows.append((round(time.monotonic() - t0, 3), name, p.x + ox, p.y + oy))
        return cb

    for name in OFFSETS:
        node.create_subscription(Odometry, f"/{name}/odom", make_cb(name), 10)
    end = t0 + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "robot", "x", "y"])
        w.writerows(rows)
    print(f"recorded {len(rows)} samples -> {args.out}")


def analyze(args):
    with open(args.csv, newline="") as f:
        rows = [(float(r["t"]), r["robot"], float(r["x"]), float(r["y"])) for r in csv.DictReader(f)]
    if len(rows) < 10:
        print(f"only {len(rows)} samples: nothing to analyze")
        return
    thr = 2 * args.radius
    r = analyze_rows(rows, thr)
    print(f"robots seen: {r['robots']}   samples: {r['samples']}")
    if r["min_at"]:
        t, a, b = r["min_at"]
        print(f"MIN SEPARATION : {r['min_d']:.3f} m at t={t:.1f}s between {a} and {b}")
    secs, mins, any_s = pair_breaches(rows, thr)
    print(f"time any pair below {thr:.2f} m : {any_s:.1f} s")
    for pair in sorted(mins):
        print(f"  pair {pair[0]}-{pair[1]}: min {mins[pair]:.3f} m, below {thr:.2f} m for {secs.get(pair, 0.0):.1f} s")
    for name in r["robots"]:
        fx, fy = r["final"][name]
        print(f"  {name}: travelled {r['travelled'].get(name, 0):.1f} m, ended at ({fx:.1f}, {fy:.1f})")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    p1 = sub.add_parser("record")
    p1.add_argument("--duration", type=float, default=40.0)
    p1.add_argument("--out", default="/tmp/smoke_odom.csv")
    p2 = sub.add_parser("analyze")
    p2.add_argument("--csv", default="/tmp/smoke_odom.csv")
    p2.add_argument("--radius", type=float, default=0.35)
    args = ap.parse_args()
    (record if args.cmd == "record" else analyze)(args)
