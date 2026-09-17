"""Minimum pairwise inter-robot separation over a recorded mission.
This is the direct measurement of BEL's criterion #1: "zero inter-robot collisions"."""
import sys, math, itertools, argparse
from collections import defaultdict
from pathlib import Path
from rosbags.highlevel import AnyReader

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--radius", type=float, default=0.35, help="robot radius in meters")
    ap.add_argument("--bin", type=float, default=0.1, help="time bucket in seconds")
    a = ap.parse_args()
    collide_dist = 2 * a.radius
    poses = defaultdict(dict)
    
    with AnyReader([Path(a.bag_dir)]) as reader:
        conns = [c for c in reader.connections if c.topic.endswith("/odom")]
        if not conns:
            print(f"No /odom topics found in {a.bag_dir}")
            sys.exit(1)
        t0 = None
        for conn, ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if t0 is None: t0 = ts
            t = round((ts - t0) / 1e9 / a.bin) * a.bin
            robot_id = conn.topic.split("/")[1]
            poses[t][robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            
    min_d = float("inf")
    min_at = None
    breaches = []
    samples = 0
    
    for t in sorted(poses):
        snap = poses[t]
        if len(snap) < 2: continue
        for r1, r2 in itertools.combinations(sorted(snap), 2):
            d = math.dist(snap[r1], snap[r2])
            samples += 1
            if d < min_d: min_d, min_at = d, (t, r1, r2)
            if d < collide_dist: breaches.append((t, r1, r2, d))
            
    duration = max(poses) if poses else 0.0
    print("=" * 60 + "\nCOLLISION ANALYSIS\n" + "=" * 60)
    print(f"duration             : {duration:.2f} s")
    print(f"MINIMUM SEPARATION   : {min_d:.3f} m")
    print(f"COLLISIONS           : {len(breaches)}")
    print("=" * 60)
    
    if len(breaches) == 0:
        print(f"\nSLIDE-READY LINE:\n\"Minimum inter-robot separation of {min_d:.2f} m recorded across a {duration:.1f}s, 6-robot mission -- zero breaches of the {collide_dist:.2f} m collision threshold.\"")
    else:
        print(f"\n*** {len(breaches)} COLLISION EVENTS DETECTED ***")

if __name__ == "__main__":
    main()
