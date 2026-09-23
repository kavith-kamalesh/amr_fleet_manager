"""
benchmark_stop_and_wait_vs_hybrid.py

Headless benchmark comparing two intersection-negotiation strategies over
the same grid graph, the same A* planner, and the same set of randomized
scenarios:

  STOP_AND_WAIT  -- static right-of-way: whichever robot is already
                    mid-edge keeps going; an entering robot always yields;
                    simultaneous-arrival ties break on robot ID. No
                    priority, no aging, no rerouting, ever. This is the
                    literal naive mutual-exclusion baseline the PS asks to
                    be measured against, and it CAN deadlock on symmetric
                    conflicts -- that is reported as a timeout, not hidden.

  HYBRID         -- mirrors amr_fleet_manager/spatial_mutex.py exactly:
                    priority-based yielding with the SAME priority-aging
                    formula (effective_priority = base + aging_rate *
                    wait_seconds), the SAME robot-id tie-break on equal
                    effective priority, and the SAME reroute-after-
                    threshold behavior via A* replanning around a blocked
                    edge. Default constants below match spatial_mutex.py's
                    ROS parameter defaults, so this benchmark measures the
                    algorithm actually deployed, not a hand-tuned variant.

Both strategies also enforce a geometric last-resort safety margin
(MIN_SAFE_DIST below) independent of the topological reservation. This
matters: an earlier version of this benchmark without it showed thousands
of "collisions" per run, tracing back to a genuine gap in the underlying
algorithm -- the topological mutex only governs who may ENTER a contested
edge, it never checks whether an uncontested, already-moving robot is
about to arrive at a node where a DIFFERENT robot is currently just
parked (correctly blocked from entering, but still physically occupying
that space). That same gap exists in the deployed spatial_mutex.py /
waypoint_nav_node.py today -- there is no geometric backstop, because
safety_fallback.py (which is meant to be exactly that backstop) is not
currently wired to anything. Fixing that in ROS is a separate, still-open
item; this benchmark's MIN_SAFE_DIST check is what safety_fallback.py
should be doing for real.

No ROS, no matplotlib, no Gazebo. Runs anywhere with numpy:
    python3 benchmark_stop_and_wait_vs_hybrid.py [--trials N] [--seed S]

Writes a per-trial CSV (raw data, for an appendix) and prints an
aggregate summary (for the slide) to stdout.
"""

import argparse
import csv
import heapq
import numpy as np


GRID_SIZE = 5
CELL = 2.0


def node_pos(n):
    return np.array([n[0] * CELL, n[1] * CELL], dtype=float)


def neighbors(n):
    x, y = n
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            yield (nx, ny)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, blocked_edges=frozenset()):
    if start == goal:
        return [start]
    open_set = [(heuristic(start, goal), 0, start, [start])]
    visited = {}
    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        if current == goal:
            return path
        if current in visited and visited[current] <= g:
            continue
        visited[current] = g
        for nxt in neighbors(current):
            edge = (current, nxt)
            if edge in blocked_edges or (nxt, current) in blocked_edges:
                continue
            ng = g + 1
            heapq.heappush(open_set, (ng + heuristic(nxt, goal), ng, nxt, path + [nxt]))
    return None


SPEED = 1.0
DT = 0.05
ROBOT_RADIUS = 0.3
RESERVATION_BUFFER_SEC = 0.6
REROUTE_WAIT_THRESHOLD_SEC = 2.0
PRIORITY_AGING_RATE = 0.05
TIMEOUT_SEC = 60.0
MIN_SAFE_DIST = ROBOT_RADIUS * 2 + 0.1


class SimRobot:
    def __init__(self, rid, start_node, goal_node, base_priority):
        self.id = rid
        self.base_priority = base_priority
        self.path = astar(start_node, goal_node)
        self.path_idx = 0
        self.pos = node_pos(start_node).astype(float)
        self.edge_progress = 0.0
        self.wait_start_time = None
        self.total_wait_time = 0.0
        self.reroute_count = 0
        self.arrived = False
        self.arrival_time = None
        self.blocked_edges = set()

    def current_edge(self):
        if self.path is None or self.path_idx >= len(self.path) - 1:
            return None
        return (self.path[self.path_idx], self.path[self.path_idx + 1])

    def edge_time_window(self, t_now):
        edge_duration = CELL / SPEED
        t_start = t_now - self.edge_progress * edge_duration
        t_end = t_start + edge_duration
        return (t_start - RESERVATION_BUFFER_SEC, t_end + RESERVATION_BUFFER_SEC)

    def effective_priority(self, t_now):
        if self.wait_start_time is None:
            return self.base_priority
        return self.base_priority + PRIORITY_AGING_RATE * (t_now - self.wait_start_time)


def windows_conflict(w1, w2):
    return not (w1[1] < w2[0] or w2[1] < w1[0])


def stop_and_wait_winner(self_r, other_r, t_now):
    if other_r.edge_progress > 0 and self_r.edge_progress == 0:
        return True
    if self_r.edge_progress > 0 and other_r.edge_progress == 0:
        return False
    return other_r.id < self_r.id


def hybrid_winner(self_r, other_r, t_now):
    self_eff = self_r.effective_priority(t_now)
    other_eff = other_r.effective_priority(t_now)
    if other_eff > self_eff:
        return True
    if other_eff < self_eff:
        return False
    return other_r.id < self_r.id


def run_scenario(robots_config, strategy, timeout_sec=TIMEOUT_SEC):
    robots = [SimRobot(i, s, g, p) for i, (s, g, p) in enumerate(robots_config)]
    winner_fn = stop_and_wait_winner if strategy == 'stop_and_wait' else hybrid_winner
    allow_reroute = (strategy == 'hybrid')

    collisions = 0
    t = 0.0
    seen_collision_pairs_this_tick = set()

    while t < timeout_sec:
        t += DT
        any_moving = False

        for robot in robots:
            if robot.arrived:
                continue
            edge = robot.current_edge()
            if edge is None:
                robot.arrived = True
                robot.arrival_time = t
                continue
            any_moving = True

            my_nodes = set(edge)
            my_window = robot.edge_time_window(t)

            blocked_by = None
            for other in robots:
                if other.id == robot.id or other.arrived:
                    continue
                other_edge = other.current_edge()
                if other_edge is None:
                    continue
                other_nodes = set(other_edge)
                if not (my_nodes & other_nodes):
                    continue
                other_window = other.edge_time_window(t)
                if windows_conflict(my_window, other_window):
                    if winner_fn(robot, other, t):
                        blocked_by = other
                        break

            geometrically_blocked = False
            if blocked_by is None:
                n1, n2 = edge
                p1, p2 = node_pos(n1), node_pos(n2)
                prospective_progress = min(1.0, robot.edge_progress + (SPEED * DT) / CELL)
                prospective_pos = p1 + (p2 - p1) * prospective_progress

                for other in robots:
                    if other.id == robot.id or other.arrived:
                        continue
                    if np.linalg.norm(prospective_pos - other.pos) < MIN_SAFE_DIST:
                        geometrically_blocked = True
                        break

            if blocked_by is not None or geometrically_blocked:
                if robot.wait_start_time is None:
                    robot.wait_start_time = t
                robot.total_wait_time += DT

                if allow_reroute and (t - robot.wait_start_time) > REROUTE_WAIT_THRESHOLD_SEC:
                    current_node = robot.path[robot.path_idx]
                    goal_node = robot.path[-1]
                    robot.blocked_edges.add(edge)
                    new_path = astar(current_node, goal_node, frozenset(robot.blocked_edges))
                    if new_path:
                        robot.path = new_path
                        robot.path_idx = 0
                        robot.edge_progress = 0.0
                        robot.wait_start_time = None
                        robot.reroute_count += 1
                continue

            robot.wait_start_time = None
            robot.edge_progress = prospective_progress
            robot.pos = prospective_pos

            if robot.edge_progress >= 1.0:
                robot.path_idx += 1
                robot.edge_progress = 0.0

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if robots[i].arrived or robots[j].arrived:
                    continue
                d = np.linalg.norm(robots[i].pos - robots[j].pos)
                if d < ROBOT_RADIUS * 2:
                    pair_key = (i, j, round(t, 1))
                    if pair_key not in seen_collision_pairs_this_tick:
                        collisions += 1
                        seen_collision_pairs_this_tick.add(pair_key)

        if not any_moving:
            break

    timed_out = not all(r.arrived for r in robots)
    makespan = max((r.arrival_time for r in robots if r.arrival_time is not None), default=t)
    total_wait = sum(r.total_wait_time for r in robots)
    total_reroutes = sum(r.reroute_count for r in robots)

    return {
        'strategy': strategy,
        'n_robots': len(robots),
        'collisions': collisions,
        'timed_out': timed_out,
        'makespan_sec': makespan if not timed_out else None,
        'mean_wait_sec': total_wait / len(robots),
        'total_reroutes': total_reroutes,
    }


def random_scenario(rng, n_robots):
    nodes = [(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]
    config = []
    used_starts = set()
    for i in range(n_robots):
        start = tuple(nodes[rng.integers(0, len(nodes))])
        while start in used_starts:
            start = tuple(nodes[rng.integers(0, len(nodes))])
        used_starts.add(start)

        goal = tuple(nodes[rng.integers(0, len(nodes))])
        tries = 0
        while goal == start and tries < 20:
            goal = tuple(nodes[rng.integers(0, len(nodes))])
            tries += 1

        priority = float(rng.uniform(0.1, 0.9))
        config.append((start, goal, priority))
    return config


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--out', type=str, default='benchmark_results.csv')
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    rows = []
    for trial_idx in range(args.trials):
        n_robots = int(rng.integers(3, 7))
        config = random_scenario(rng, n_robots)

        for strategy in ('stop_and_wait', 'hybrid'):
            result = run_scenario(config, strategy)
            result['trial'] = trial_idx
            rows.append(result)

    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'trial', 'strategy', 'n_robots', 'collisions', 'timed_out',
            'makespan_sec', 'mean_wait_sec', 'total_reroutes',
        ])
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    print_summary(rows, args.trials)
    print(f"\nRaw per-trial data written to {args.out}")


def print_summary(rows, n_trials):
    for strategy in ('stop_and_wait', 'hybrid'):
        srows = [r for r in rows if r['strategy'] == strategy]
        n_timeout = sum(1 for r in srows if r['timed_out'])
        n_collisions = sum(r['collisions'] for r in srows)
        completed = [r for r in srows if not r['timed_out']]
        mean_makespan = (sum(r['makespan_sec'] for r in completed) / len(completed)
                          if completed else float('nan'))
        mean_wait = sum(r['mean_wait_sec'] for r in srows) / len(srows)
        mean_reroutes = sum(r['total_reroutes'] for r in srows) / len(srows)

        print(f"\n=== {strategy} ({n_trials} trials) ===")
        print(f"  Collisions (should be 0 for both -- correctness check): {n_collisions}")
        print(f"  Timed out (deadlocked, never all arrived):              {n_timeout}/{n_trials}  ({100*n_timeout/n_trials:.1f}%)")
        print(f"  Mean makespan (completed trials only, sec):             {mean_makespan:.2f}")
        print(f"  Mean per-robot wait time (sec):                         {mean_wait:.2f}")
        print(f"  Mean reroutes per trial:                                {mean_reroutes:.2f}")

    sw = [r for r in rows if r['strategy'] == 'stop_and_wait' and not r['timed_out']]
    hy = [r for r in rows if r['strategy'] == 'hybrid' and not r['timed_out']]
    sw_by_trial = {r['trial']: r for r in sw}
    hy_by_trial = {r['trial']: r for r in hy}
    common_trials = set(sw_by_trial) & set(hy_by_trial)

    if common_trials:
        sw_mean = sum(sw_by_trial[t]['makespan_sec'] for t in common_trials) / len(common_trials)
        hy_mean = sum(hy_by_trial[t]['makespan_sec'] for t in common_trials) / len(common_trials)
        pct_improvement = 100.0 * (sw_mean - hy_mean) / sw_mean
        print(f"\n=== Head-to-head on the {len(common_trials)} trials where BOTH strategies completed ===")
        print(f"  (Caveat: this subset is biased toward EASY scenarios, since it excludes")
        print(f"   every trial where stop_and_wait deadlocked -- exactly the cases hybrid")
        print(f"   exists to handle. Read this as 'overhead on easy conflicts', not the")
        print(f"   headline number.)")
        print(f"  stop_and_wait mean makespan: {sw_mean:.2f}s")
        print(f"  hybrid mean makespan:        {hy_mean:.2f}s")
        print(f"  Difference:                  {pct_improvement:+.1f}%")
    else:
        print("\nNo trials where both strategies completed -- cannot compute this subset comparison.")

    print(f"\n=== Overall (all {n_trials} trials, timeouts penalized at {TIMEOUT_SEC:.0f}s -- the honest comparison) ===")
    for strategy in ('stop_and_wait', 'hybrid'):
        srows = [r for r in rows if r['strategy'] == strategy]
        n_timeout = sum(1 for r in srows if r['timed_out'])
        success_rate = 100.0 * (n_trials - n_timeout) / n_trials
        overall_mean = sum(
            (r['makespan_sec'] if not r['timed_out'] else TIMEOUT_SEC) for r in srows
        ) / len(srows)
        print(f"  {strategy:14s}  success rate: {success_rate:5.1f}%   "
              f"mean completion time (incl. timeout penalty): {overall_mean:6.2f}s")

    sw_all = [r for r in rows if r['strategy'] == 'stop_and_wait']
    hy_all = [r for r in rows if r['strategy'] == 'hybrid']
    sw_overall = sum((r['makespan_sec'] if not r['timed_out'] else TIMEOUT_SEC) for r in sw_all) / len(sw_all)
    hy_overall = sum((r['makespan_sec'] if not r['timed_out'] else TIMEOUT_SEC) for r in hy_all) / len(hy_all)
    overall_pct = 100.0 * (sw_overall - hy_overall) / sw_overall
    print(f"\n  Overall improvement (hybrid vs stop_and_wait, timeouts included): {overall_pct:+.1f}%  (PS target: >= 20%)")


if __name__ == '__main__':
    main()
