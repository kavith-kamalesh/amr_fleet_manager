"""
protocol_sim.py -- run the REAL per-robot protocol (fleet_protocol.FleetAgent)
over a simulated lossy radio network and audit what the robots physically do.

  python3 protocol_sim.py --trials 30 --out results
"""
import argparse
import csv
import os
import random
import statistics
import sys

try:
    import fleet_pibt_bench as fb
    from fleet_protocol import FleetAgent, manhattan
except ImportError:
    from amr_fleet_manager import fleet_pibt_bench as fb
    from amr_fleet_manager.fleet_protocol import FleetAgent, manhattan

MAX_TICKS = 200


def run_protocol_trial(n, seed, rounds=6, loss=0.0, offline=0.0, radius=None,
                       dock="vanish", nbrs=None, trial_seed=None, scenario=None,
                       rotations=True):
    """loss    : independent per-receiver message loss probability
       offline : per-robot per-tick probability that its radio is dead
                 (it neither sends nor hears, and holds still)
       radius  : radio range in grid cells (None = unlimited)
       dock    : 'vanish' docked robot leaves the grid; 'park' it stays and
                 becomes an obstacle for everyone else"""
    nbrs = nbrs or fb.build_graph()
    starts, goals, prio = scenario or fb.make_scenario(n, seed)
    n = len(starts)
    rng = random.Random((trial_seed if trial_seed is not None else seed) * 7919 + 17)
    agents = [FleetAgent(i, nbrs, prio[i], starts[i], goals[i], rounds=rounds,
                         vanish_docked=(dock == "vanish"),
                         rotations=rotations) for i in range(n)]
    for a in agents:                      # fleet roster from the launch config
        for b in agents:
            a.seed_peer(b.id, b.cell, b.goal, b.prio)
    live = {a.id: a for a in agents}
    docked_ids = set()
    finish = {}
    collisions = msgs = 0
    parked = {}                       # id -> cell (park mode)
    idle = 0                          # consecutive ticks in which nobody moved
    for tick in range(MAX_TICKS):
        if len(docked_ids) == n or not live:
            break
        off = {i for i in live if rng.random() < offline}

        def deliver(m):
            nonlocal msgs
            if m["id"] in off:
                return
            s_cell = tuple(m["cell"])
            for j, rcv in live.items():
                if j == m["id"] or j in off:
                    continue
                if radius is not None and manhattan(s_cell, rcv.cell) > radius:
                    continue
                msgs += 1
                if rng.random() >= loss:
                    rcv.receive(m)

        starts_msgs = [a.begin_tick(tick) for a in live.values()]
        for m in starts_msgs:
            deliver(m)
        for a in live.values():
            if a.id in off:
                a.target, a.committed = a.cell, True
            else:
                a.plan()
        for r in range(1, rounds + 1):
            ms = [a.round_msg(r) for a in live.values()]
            for m in ms:
                deliver(m)
            for a in live.values():
                if a.id not in off:
                    a.step_commit()
        for a in live.values():
            a.finalize()
        # ---- audit what physically happens ----
        cur = {i: a.cell for i, a in live.items()}
        nxt = {i: a.target for i, a in live.items()}
        occ = {}
        for i, c in nxt.items():
            occ.setdefault(c, []).append(i)
        collisions += sum(len(v) - 1 for v in occ.values() if len(v) > 1)
        ids = sorted(live)
        for x in range(len(ids)):
            for y in range(x + 1, len(ids)):
                i, j = ids[x], ids[y]
                if nxt[i] == cur[j] and nxt[j] == cur[i] and nxt[i] != cur[i]:
                    collisions += 1
        idle = idle + 1 if all(nxt[i] == cur[i] for i in live) else 0
        for a in list(live.values()):
            a.apply()
            if a.docked and a.id not in docked_ids:
                docked_ids.add(a.id)
                finish[a.id] = tick + 1
                if dock == "vanish":
                    del live[a.id]
                    for b in live.values():
                        b.forget(a.id)
                # park: stays in `live`, keeps broadcasting docked=True
        if idle >= 12 and offline == 0.0:
            break                     # permanent deadlock: stop early
    done = [finish.get(i, MAX_TICKS) for i in range(n)]
    return dict(success=len(finish) == n, unfinished=n - len(finish),
                collisions=collisions, mean_completion=sum(done) / n, msgs=msgs)


CONFIGS = [
    # name,                          kwargs
    ("ideal radio",                  dict()),
    ("loss 5%",                      dict(loss=0.05)),
    ("loss 10%",                     dict(loss=0.10)),
    ("loss 20%",                     dict(loss=0.20)),
    ("loss 30%",                     dict(loss=0.30)),
    ("radio dead 5% of ticks",       dict(offline=0.05)),
    ("radio dead 10% of ticks",      dict(offline=0.10)),
    ("radio dead 20% of ticks",      dict(offline=0.20)),
    ("range 6 cells",                dict(radius=6)),
    ("rounds=2",                     dict(rounds=2)),
    ("rounds=4",                     dict(rounds=4)),
    ("rounds=10",                    dict(rounds=10)),
    ("parked robots stay as obstacles", dict(dock="park")),
]


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, nargs="+", default=[24, 48])
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="results")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)
    nbrs = fb.build_graph()
    rows = []
    print(f"{'robots':>6} {'condition':<34}{'success%':>9}{'coll':>6}{'mean_t':>8}{'unfin':>7}{'msgs/trial':>11}")
    for n in a.n:
        for name, kw in CONFIGS:
            rs = [run_protocol_trial(n, a.seed * 100003 + n * 1009 + k, nbrs=nbrs,
                                     trial_seed=k, **kw) for k in range(a.trials)]
            row = dict(n_robots=n, condition=name, trials=a.trials,
                       success_pct=100.0 * sum(r["success"] for r in rs) / a.trials,
                       collisions=sum(r["collisions"] for r in rs),
                       mean_completion=statistics.mean(r["mean_completion"] for r in rs),
                       mean_unfinished=statistics.mean(r["unfinished"] for r in rs),
                       msgs_per_trial=statistics.mean(r["msgs"] for r in rs))
            rows.append(row)
            print(f"{n:>6} {name:<34}{row['success_pct']:>9.1f}{row['collisions']:>6}"
                  f"{row['mean_completion']:>8.1f}{row['mean_unfinished']:>7.2f}"
                  f"{row['msgs_per_trial']:>11.0f}")
    path = os.path.join(a.out, "protocol_summary.csv")
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\nWrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
