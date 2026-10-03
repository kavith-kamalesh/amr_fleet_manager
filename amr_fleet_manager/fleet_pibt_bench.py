"""
fleet_pibt_bench.py -- headless, reproducible fleet-coordination benchmark.

Compares three policies on the SAME random scenarios:
  stop_and_wait : each robot follows its own shortest path and halts whenever
                  the next cell is occupied/claimed (the PS baseline).
  central       : prioritised space-time A* with a global reservation table
                  (what a deployed central fleet manager runs). Strong baseline.
  decentral     : decentralised step resolution using PRIORITY INHERITANCE and
                  PRIORITY AGING (PIBT, Okumura et al. 2019). Each robot only
                  negotiates with the neighbour that occupies the cell it wants.

Model: robots move one grid cell per tick; a robot leaves the grid when it
reaches its goal (dock). Collision = two robots in one cell, or a head-on swap.

Usage:
  python3 fleet_pibt_bench.py --sizes 3 6 12 24 48 --trials 30 --out results
"""
import argparse
import csv
import heapq
import os
import random
import statistics
import sys
from collections import deque

try:                                  # run as script / from tests
    from pibt_core import bfs_dist, grid_graph, pibt_step
except ImportError:                   # run as an installed ROS package
    from amr_fleet_manager.pibt_core import bfs_dist, grid_graph, pibt_step

GRID = 12
MAX_STEPS = 400
AGING_WEIGHT = 5.0
AGING_MODE = "static"   # static (PIBT standard) | monotone | reset  -- see notes
DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1)]
POLICIES = ["stop_and_wait", "central", "decentral"]


# ----------------------------------------------------------------- graph ---
def build_graph(n=GRID):
    return grid_graph(n)


def make_scenario(n_robots, seed, n=GRID):
    rng = random.Random(seed)
    nodes = sorted(build_graph(n).keys())
    starts = rng.sample(nodes, n_robots)
    goals = rng.sample(nodes, n_robots)
    for i in range(n_robots):
        while goals[i] == starts[i]:
            goals[i] = rng.choice(nodes)
    prio = [rng.random() for _ in range(n_robots)]
    return starts, goals, prio


# ------------------------------------------------------------- policies ----
def next_hop(nbrs, dist, cur):
    return min(nbrs[cur], key=lambda u: (dist[u], u))


def policy_stop_and_wait(S):
    occupied = {S["cur"][i]: i for i in S["active"]}
    claimed = set()
    nxt = {}
    for i in sorted(S["active"]):
        c = S["cur"][i]
        t = next_hop(S["nbrs"], S["dist"][i], c)
        if t in occupied or t in claimed:
            nxt[i] = c
            claimed.add(c)
        else:
            nxt[i] = t
            claimed.add(t)
    return nxt


def _plan_in_order(S, order, blocked=()):
    nbrs = S["nbrs"]
    vres, eres = set(), set()
    for i in S["active"]:
        vres.add((S["cur"][i], 0))
    for c in blocked:                      # broken robots: cell blocked forever
        for tt in range(MAX_STEPS + 2):
            vres.add((c, tt))
    plans = {}
    for i in order:
        start, goal, dist = S["cur"][i], S["goals"][i], S["dist"][i]
        open_ = [(dist[start], 0, start, 0)]
        parent = {(start, 0): None}
        found = None
        while open_:
            f, g, v, t = heapq.heappop(open_)
            if v == goal:
                found = (v, t)
                break
            if t >= MAX_STEPS:
                continue
            for u in nbrs[v] + [v]:
                st = (u, t + 1)
                if st in parent or st in vres:
                    continue
                if (u, v, t) in eres:       # swap with a planned robot
                    continue
                parent[st] = (v, t)
                heapq.heappush(open_, (g + 1 + dist[u], g + 1, u, t + 1))
        if found is None:
            return plans, i
        path, s_ = [], found
        while s_ is not None:
            path.append(s_[0])
            s_ = parent[s_]
        path.reverse()
        plans[i] = path
        for t, v in enumerate(path):
            vres.add((v, t))
            if t + 1 < len(path):
                eres.add((v, path[t + 1], t))
    return plans, None


def plan_central(S, blocked=()):
    """Prioritised space-time A*. A robot that fails is promoted to the front
    and everything is replanned. A robot that still fails as top priority has
    an unreachable goal: it is dropped (plan None) without blocking the rest."""
    order = sorted((i for i in S["active"] if i not in S["broken"]),
                   key=lambda i: -S["prio"][i])
    S["replans"] = 0
    hopeless = set()
    plans = {}
    for _ in range(2 * len(order) + 2):
        todo = [i for i in order if i not in hopeless]
        obst = set(blocked) | {S["cur"][h] for h in hopeless}
        plans, failed = _plan_in_order(S, todo, obst)
        if failed is None:
            break
        S["replans"] += 1
        if todo and failed == todo[0]:
            hopeless.add(failed)
        else:
            order.remove(failed)
            order.insert(0, failed)
    else:
        plans = {}
    for i in order:
        plans.setdefault(i, None)
    return plans


def _central_dead_follow(S):
    """Planner is down: keep the cached ROUTE, arbitrate locally with the
    stop-and-wait rule (move only if the next cell is free and unclaimed)."""
    if "routes" not in S:
        S["routes"], S["rptr"] = {}, {}
        k = S["t"] - S["plan_t0"]
        for i in S["active"]:
            p = S["plans"].get(i)
            if p:
                seg = p[min(k, len(p) - 1):]
                route = [seg[0]]
                for v in seg[1:]:
                    if v != route[-1]:
                        route.append(v)
            else:
                route = [S["cur"][i]]
            S["routes"][i], S["rptr"][i] = route, 0
    occupied = {S["cur"][i] for i in S["active"]}
    claimed, nxt = set(), {}
    for i in sorted(S["active"]):
        c = S["cur"][i]
        r = S["routes"].get(i, [c])
        ptr = S["rptr"].get(i, 0)
        t = r[ptr + 1] if ptr + 1 < len(r) else c
        if t != c and (t in occupied or t in claimed):
            nxt[i] = c
            claimed.add(c)
        else:
            nxt[i] = t
            claimed.add(t)
            if t != c:
                S["rptr"][i] = ptr + 1
    return nxt


def policy_central(S):
    if "plans" not in S:
        S["plans"] = plan_central(S)
        S["plan_t0"] = 0
        S["plan_fail"] = sum(1 for p in S["plans"].values() if p is None)
    if S["dead"]:
        return _central_dead_follow(S)
    if S["replan"]:
        blocked = {S["cur"][i] for i in S["broken"]}
        S["plans"] = plan_central(S, blocked)
        S["plan_t0"] = S["t"]
        S["plan_fail"] = sum(1 for p in S["plans"].values() if p is None)
        S["replan"] = False
    k = S["t"] - S["plan_t0"]
    nxt = {}
    for i in S["active"]:
        p = S["plans"].get(i)
        nxt[i] = p[k + 1] if (p and k + 1 < len(p)) else S["cur"][i]
    return nxt


def policy_decentral(S):
    """PIBT step. Silent (dropped-out) and broken robots hold still."""
    eff = {i: AGING_WEIGHT * S["prio"][i] + S["wait"][i] for i in S["active"]}
    return pibt_step(S["nbrs"], S["cur"], S["dist"], eff, S["active"],
                     S["silent"] | S["broken"])


POLICY_FN = {
    "stop_and_wait": policy_stop_and_wait,
    "central": policy_central,
    "decentral": policy_decentral,
}


# -------------------------------------------------------------- simulator --
def run_trial(policy, scenario, dropout=0.0, seed=0, nbrs=None, trace=None,
              events=None):
    starts, goals, prio = scenario
    nbrs = nbrs or build_graph()
    n = len(starts)
    cache = {}
    dists = []
    for g in goals:
        if g not in cache:
            cache[g] = bfs_dist(nbrs, g)
        dists.append(cache[g])
    S = dict(nbrs=nbrs, cur=list(starts), goals=goals, prio=prio, dist=dists,
             active=set(range(n)), wait=[0] * n, silent=set(), t=0,
             broken=set(), dead=False, replan=False)
    ev = events or {}
    ever_broken = set()
    infeasible = set()
    finish = [None] * n
    collisions = 0
    rng = random.Random(seed * 7919 + 13)
    fn = POLICY_FN[policy]

    def _snap():
        if trace is not None:
            trace.append([list(S["cur"][i]) if i in S["active"] else None
                          for i in range(n)])
    _snap()
    for t in range(MAX_STEPS):
        if not (S["active"] - S["broken"]):
            break
        S["t"] = t
        if ev.get("broken") and t == ev.get("break_tick"):
            S["broken"] = {i for i in ev["broken"] if i in S["active"]}
            ever_broken = set(S["broken"])
            frozen = {S["cur"][i] for i in S["broken"]}
            infeasible = {i for i in S["active"] - S["broken"] if goals[i] in frozen}
            remap = {}
            for i in range(n):
                if i in infeasible or goals[i] in frozen:
                    continue                  # keep old gradient: waits at the bay
                if goals[i] not in remap:
                    remap[goals[i]] = bfs_dist(nbrs, goals[i], frozen, fill=10 ** 6)
                S["dist"][i] = remap[goals[i]]
            S["replan"] = True
        dead_at = ev.get("planner_dead_tick")
        S["dead"] = dead_at is not None and t >= dead_at
        S["silent"] = {i for i in S["active"] if rng.random() < dropout}
        nxt = fn(S)
        for i in S["broken"]:
            nxt[i] = S["cur"][i]
        # ---- safety audit (independent of the policy) ----
        seen = {}
        for i in S["active"]:
            v = nxt[i]
            if v in seen:
                collisions += 1
            seen[v] = i
        for i in S["active"]:
            for j in S["active"]:
                if i < j and nxt[i] == S["cur"][j] and nxt[j] == S["cur"][i] \
                        and nxt[i] != S["cur"][i]:
                    collisions += 1
        # ---- apply ----
        for i in list(S["active"]):
            before = S["dist"][i][S["cur"][i]]
            S["cur"][i] = nxt[i]
            progressed = S["dist"][i][S["cur"][i]] < before
            if AGING_MODE == "static":
                S["wait"][i] += 1
            elif AGING_MODE == "reset":
                S["wait"][i] = 0 if progressed else S["wait"][i] + 1
            elif not progressed:
                S["wait"][i] += 1     # monotone: never reset
            if S["cur"][i] == goals[i]:
                finish[i] = t + 1
                S["active"].discard(i)
        _snap()
    targets = [i for i in range(n) if i not in ever_broken and i not in infeasible]
    done = [finish[i] if finish[i] is not None else MAX_STEPS for i in targets]
    return dict(
        success=all(finish[i] is not None for i in targets),
        unfinished=sum(1 for i in targets if finish[i] is None),
        collisions=collisions,
        mean_completion=sum(done) / len(targets) if targets else 0.0,
        makespan=max(done) if done else 0,
        plan_fail=S.get("plan_fail", 0),
    )


# ------------------------------------------------------------ experiments --
def scalability(sizes, trials, seed):
    nbrs = build_graph()
    rows = []
    for n in sizes:
        for k in range(trials):
            sc = make_scenario(n, seed * 100003 + n * 1009 + k)
            for p in POLICIES:
                r = run_trial(p, sc, nbrs=nbrs)
                rows.append(dict(n_robots=n, trial=k, policy=p, **r))
    return rows


def dropout_sweep(n, rates, trials, seed):
    nbrs = build_graph()
    rows = []
    for rate in rates:
        for k in range(trials):
            sc = make_scenario(n, seed * 100003 + n * 1009 + k)
            r = run_trial("decentral", sc, dropout=rate, seed=k, nbrs=nbrs)
            rows.append(dict(n_robots=n, dropout=rate, trial=k, **r))
    return rows


FAIL_CASES = [
    # name,                       policy,      breakdown, planner_dead_tick
    ("central_live",              "central",   True,  None),
    ("central_dead",              "central",   True,  3),
    ("decentral",                 "decentral", True,  None),
    ("central_dead_no_breakdown", "central",   False, 3),
]
BREAK_TICK = 3


def make_events(sc, seed, breakdown, dead_tick):
    """One robot with a long route (>= 8 cells) freezes for good at tick 3."""
    if not breakdown:
        return dict(broken=[], break_tick=None, planner_dead_tick=dead_tick)
    rng = random.Random(seed * 31337 + 7)
    cands = [i for i, (a, g) in enumerate(zip(sc[0], sc[1]))
             if abs(a[0] - g[0]) + abs(a[1] - g[1]) >= 8]
    return dict(broken=[rng.choice(cands)] if cands else [],
                break_tick=BREAK_TICK, planner_dead_tick=dead_tick)


def failure_experiment(n, trials, seed):
    nbrs = build_graph()
    rows = []
    for k in range(trials):
        sc = make_scenario(n, seed * 100003 + n * 1009 + k)
        for name, pol, brk, dead in FAIL_CASES:
            ev = make_events(sc, k, brk, dead)
            r = run_trial(pol, sc, nbrs=nbrs, events=ev)
            rows.append(dict(n_robots=n, trial=k, case=name, **r))
    return rows


def summarize_failure(rows):
    out = []
    for n in sorted({r["n_robots"] for r in rows}):
        for name, _, _, _ in FAIL_CASES:
            sub = [r for r in rows if r["n_robots"] == n and r["case"] == name]
            out.append(dict(
                n_robots=n, case=name, trials=len(sub),
                success_pct=100.0 * sum(r["success"] for r in sub) / len(sub),
                collisions=sum(r["collisions"] for r in sub),
                mean_completion=statistics.mean(r["mean_completion"] for r in sub),
                mean_unfinished=statistics.mean(r["unfinished"] for r in sub),
            ))
    return out


def summarize(rows):
    out = []
    sizes = sorted({r["n_robots"] for r in rows})
    for n in sizes:
        base = {r["trial"]: r for r in rows
                if r["n_robots"] == n and r["policy"] == "stop_and_wait"}
        for p in POLICIES:
            sub = [r for r in rows if r["n_robots"] == n and r["policy"] == p]
            m = statistics.mean(r["mean_completion"] for r in sub)
            bm = statistics.mean(b["mean_completion"] for b in base.values())
            both = [r for r in sub if r["success"] and base[r["trial"]]["success"]]
            if both:
                mb = statistics.mean(r["mean_completion"] for r in both)
                bb = statistics.mean(base[r["trial"]]["mean_completion"] for r in both)
                subset_gain = (bb - mb) / bb * 100
            else:
                subset_gain = float("nan")
            out.append(dict(
                n_robots=n, policy=p, trials=len(sub),
                success_pct=100.0 * sum(r["success"] for r in sub) / len(sub),
                collisions=sum(r["collisions"] for r in sub),
                mean_completion=m,
                gain_vs_stop_and_wait_pct=(bm - m) / bm * 100,
                gain_on_commonly_solved_pct=subset_gain,
                commonly_solved=len(both),
                plan_fail=sum(r["plan_fail"] for r in sub),
            ))
    return out


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def print_table(summary):
    hdr = (f"{'robots':>6} {'policy':<14}{'success%':>9}{'coll':>6}"
           f"{'mean_t':>8}{'gain%(pen)':>12}{'gain%(common)':>15}{'n_common':>9}{'planfail':>9}")
    print(hdr)
    print("-" * len(hdr))
    for s in summary:
        print(f"{s['n_robots']:>6} {s['policy']:<14}{s['success_pct']:>9.1f}"
              f"{s['collisions']:>6}{s['mean_completion']:>8.1f}"
              f"{s['gain_vs_stop_and_wait_pct']:>12.1f}"
              f"{s['gain_on_commonly_solved_pct']:>15.1f}"
              f"{s['commonly_solved']:>9}{s['plan_fail']:>9}")


def plot(summary, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib missing -> skipping chart")
        return
    sizes = sorted({s["n_robots"] for s in summary})
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    style = {"stop_and_wait": ("tab:red", "Stop-and-wait"),
             "central": ("tab:blue", "Central planner"),
             "decentral": ("tab:green", "Decentralised (ours)")}
    for p, (c, label) in style.items():
        ys = [next(s for s in summary if s["n_robots"] == n and s["policy"] == p)
              for n in sizes]
        ax[0].plot(sizes, [y["success_pct"] for y in ys], "o-", color=c, label=label)
        ax[1].plot(sizes, [y["mean_completion"] for y in ys], "o-", color=c, label=label)
    ax[0].set(title="Trials fully completed (%)", xlabel="robots", ylim=(0, 105))
    ax[1].set(title="Mean task completion (ticks, timeouts penalised)", xlabel="robots")
    ax[0].legend()
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=160)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--sizes", type=int, nargs="+", default=[3, 6, 12, 24, 48])
    ap.add_argument("--trials", type=int, default=30)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--out", default="results")
    a = ap.parse_args(argv)
    os.makedirs(a.out, exist_ok=True)

    rows = scalability(a.sizes, a.trials, a.seed)
    write_csv(os.path.join(a.out, "scalability_trials.csv"), rows)
    summary = summarize(rows)
    write_csv(os.path.join(a.out, "scalability_summary.csv"), summary)
    print(f"\n== Scalability ({a.trials} trials/size, seed {a.seed}, "
          f"{GRID}x{GRID} grid, timeout {MAX_STEPS} ticks) ==")
    print_table(summary)
    plot(summary, os.path.join(a.out, "scalability.png"))

    n_d = 24 if 24 in a.sizes else a.sizes[-1]
    drows = dropout_sweep(n_d, [0.0, 0.05, 0.10, 0.20, 0.30], a.trials, a.seed)
    write_csv(os.path.join(a.out, "dropout_trials.csv"), drows)
    print(f"\n== Comms-dropout robustness (decentral, {n_d} robots) ==")
    print(f"{'dropout':>8}{'success%':>10}{'coll':>6}{'mean_t':>8}")
    for rate in sorted({r["dropout"] for r in drows}):
        sub = [r for r in drows if r["dropout"] == rate]
        print(f"{rate:>8.2f}{100.0 * sum(r['success'] for r in sub) / len(sub):>10.1f}"
              f"{sum(r['collisions'] for r in sub):>6}"
              f"{statistics.mean(r['mean_completion'] for r in sub):>8.1f}")
    frows = []
    for n in [x for x in (24, 48) if x in a.sizes]:
        frows += failure_experiment(n, a.trials, a.seed)
    if frows:
        write_csv(os.path.join(a.out, "failure_trials.csv"), frows)
        fsum = summarize_failure(frows)
        write_csv(os.path.join(a.out, "failure_summary.csv"), fsum)
        print(f"\n== Robot breakdown at tick {BREAK_TICK} (1 robot frozen for good); "
              f"success = every OTHER robot docked ==")
        print(f"{'robots':>6} {'case':<27}{'success%':>9}{'coll':>6}{'mean_t':>8}{'unfinished':>11}")
        for f in fsum:
            print(f"{f['n_robots']:>6} {f['case']:<27}{f['success_pct']:>9.1f}"
                  f"{f['collisions']:>6}{f['mean_completion']:>8.1f}{f['mean_unfinished']:>11.2f}")
    print(f"\nWrote CSVs + scalability.png to ./{a.out}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
