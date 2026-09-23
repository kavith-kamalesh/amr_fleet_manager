"""
benchmark_scalability.py

Tests whether neighbor-radius filtering (the "filter neighbours by zone"
fix) reduces conflict-check cost at scale without breaking correctness,
and reports how conflict-check cost and message volume actually grow
with fleet size.

Two different things get measured and are NOT conflated:
  - CONFLICT-CHECK COST: how many pairwise conflict evaluations
    spatial_mutex.mutex_loop() performs. This is what neighbor-radius
    filtering actually reduces (a robot only evaluates peers within
    NEIGHBOR_RADIUS of its own current position, instead of every peer
    in the fleet). This is a real CPU-cost fix.
  - MESSAGE VOLUME (messages/sec): every robot still broadcasts its own
    intent to everyone on /fleet/spatial_intent at 10Hz regardless of
    filtering -- filtering changes what a robot PAYS ATTENTION TO, not
    what it PUBLISHES. Actually reducing broadcast volume needs DDS-level
    scoped multicast/partitions, which is a separate, real change not
    implemented here. Reported as n_robots * 10Hz, honestly labeled as
    unaffected by this fix.

Filtering design: distance-radius filter (skip a peer if its current
edge midpoint is farther than NEIGHBOR_RADIUS from self's), not a grid-
zone partition -- this avoids zone-boundary edge cases and matches the
NEIGHBOR_DIST pattern already used in sim2d.py in this repo.

NEIGHBOR_RADIUS is chosen generously (4x the grid cell size) specifically
so it CANNOT be the thing that makes collisions disappear -- if filtering
ever produces a collision that the unfiltered baseline didn't, that is
reported, not hidden.

Grid is enlarged to 12x12 (from the 5x5 used in earlier benchmarks) for
this test specifically, because 24 robots on a 5x5=25-node grid would be
pathologically saturated -- a bigger grid is needed for the fleet-size
scaling question to mean anything.

Usage: python3 benchmark_scalability.py [--seed S] [--trials-per-size N]
"""

import argparse
import heapq
import numpy as np


GRID_SIZE = 12
CELL = 2.0
NEIGHBOR_RADIUS = 4 * CELL  # deliberately generous -- see module docstring


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
TIMEOUT_SEC = 90.0  # larger grid, allow more time
MIN_SAFE_DIST = ROBOT_RADIUS * 2 + 0.1
BROADCAST_HZ = 10.0  # matches spatial_mutex's 0.1s mutex_loop timer


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

    def edge_midpoint(self):
        edge = self.current_edge()
        if edge is None:
            return self.pos
        n1, n2 = edge
        return (node_pos(n1) + node_pos(n2)) / 2.0

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


def hybrid_winner(self_r, other_r, t_now):
    self_eff = self_r.effective_priority(t_now)
    other_eff = other_r.effective_priority(t_now)
    if other_eff > self_eff:
        return True
    if other_eff < self_eff:
        return False
    return other_r.id < self_r.id


def run_scenario(robots_config, use_filter, timeout_sec=TIMEOUT_SEC):
    """robots_config: list of (start_node, goal_node, base_priority).
    use_filter: if True, a robot only evaluates peers within
    NEIGHBOR_RADIUS of its own current edge midpoint for conflict
    checking (both topological and geometric-safety checks)."""
    robots = [SimRobot(i, s, g, p) for i, (s, g, p) in enumerate(robots_config)]

    collisions = 0
    conflict_checks_performed = 0
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
            my_midpoint = robot.edge_midpoint()

            blocked_by = None
            for other in robots:
                if other.id == robot.id or other.arrived:
                    continue
                other_edge = other.current_edge()
                if other_edge is None:
                    continue

                if use_filter:
                    dist = np.linalg.norm(my_midpoint - other.edge_midpoint())
                    if dist > NEIGHBOR_RADIUS:
                        continue  # filtered out -- never evaluated further

                conflict_checks_performed += 1

                other_nodes = set(other_edge)
                if not (my_nodes & other_nodes):
                    continue
                other_window = other.edge_time_window(t)
                if windows_conflict(my_window, other_window):
                    if hybrid_winner(robot, other, t):
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
                    if use_filter:
                        dist = np.linalg.norm(prospective_pos - other.pos)
                        if dist > NEIGHBOR_RADIUS:
                            continue
                    if np.linalg.norm(prospective_pos - other.pos) < MIN_SAFE_DIST:
                        geometrically_blocked = True
                        break

            if blocked_by is not None or geometrically_blocked:
                if robot.wait_start_time is None:
                    robot.wait_start_time = t
                robot.total_wait_time += DT
                if (t - robot.wait_start_time) > REROUTE_WAIT_THRESHOLD_SEC:
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

            n1, n2 = edge
            p1, p2 = node_pos(n1), node_pos(n2)
            prospective_progress = min(1.0, robot.edge_progress + (SPEED * DT) / CELL)
            robot.wait_start_time = None
            robot.edge_progress = prospective_progress
            robot.pos = p1 + (p2 - p1) * prospective_progress
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

    n_timeout = sum(1 for r in robots if r.arrival_time is None)
    makespan = max((r.arrival_time for r in robots if r.arrival_time is not None), default=t)

    return {
        'collisions': collisions,
        'timed_out': n_timeout > 0,
        'n_timeout': n_timeout,
        'makespan': makespan,
        'conflict_checks': conflict_checks_performed,
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
    parser.add_argument('--trials-per-size', type=int, default=15)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    fleet_sizes = [3, 6, 12, 24]
    rng = np.random.default_rng(args.seed)

    print(f"Grid: {GRID_SIZE}x{GRID_SIZE} ({GRID_SIZE*GRID_SIZE} nodes)  "
          f"NEIGHBOR_RADIUS={NEIGHBOR_RADIUS}m ({NEIGHBOR_RADIUS/CELL:.0f} cells)\n")

    print(f"{'n_robots':>8} | {'msgs/sec':>9} | {'mode':>10} | {'collisions':>10} | "
          f"{'timeouts':>9} | {'mean_makespan':>13} | {'conflict_checks/trial':>22}")
    print("-" * 100)

    results = []
    for n_robots in fleet_sizes:
        scenarios = [random_scenario(rng, n_robots) for _ in range(args.trials_per_size)]
        msgs_per_sec = n_robots * BROADCAST_HZ

        for use_filter in (False, True):
            mode = 'filtered' if use_filter else 'unfiltered'
            outcomes = [run_scenario(config, use_filter) for config in scenarios]

            total_collisions = sum(o['collisions'] for o in outcomes)
            total_timeouts = sum(1 for o in outcomes if o['timed_out'])
            mean_makespan = sum(o['makespan'] for o in outcomes) / len(outcomes)
            mean_conflict_checks = sum(o['conflict_checks'] for o in outcomes) / len(outcomes)

            print(f"{n_robots:>8} | {msgs_per_sec:>9.0f} | {mode:>10} | {total_collisions:>10} | "
                  f"{total_timeouts:>3}/{args.trials_per_size:<5} | {mean_makespan:>13.2f} | {mean_conflict_checks:>22.0f}")

            results.append({
                'n_robots': n_robots, 'mode': mode, 'collisions': total_collisions,
                'timeouts': total_timeouts, 'mean_makespan': mean_makespan,
                'mean_conflict_checks': mean_conflict_checks,
            })
        print()

    print("=== Summary ===")
    any_collision = any(r['collisions'] > 0 for r in results)
    print(f"Collisions in ANY configuration (correctness check): {'YES -- INVESTIGATE' if any_collision else 'None -- filtering did not compromise safety'}")

    print("\nConflict-check cost reduction from filtering (CPU-cost proxy, not message volume):")
    for n_robots in fleet_sizes:
        unf = next(r for r in results if r['n_robots'] == n_robots and r['mode'] == 'unfiltered')
        filt = next(r for r in results if r['n_robots'] == n_robots and r['mode'] == 'filtered')
        if unf['mean_conflict_checks'] > 0:
            reduction = 100.0 * (1 - filt['mean_conflict_checks'] / unf['mean_conflict_checks'])
        else:
            reduction = 0.0
        print(f"  n_robots={n_robots:>3}: unfiltered={unf['mean_conflict_checks']:>8.0f}  "
              f"filtered={filt['mean_conflict_checks']:>8.0f}  reduction={reduction:>5.1f}%")

    print("\nMessage volume (broadcast /fleet/spatial_intent at 10Hz per active robot -- "
          "UNAFFECTED by neighbor filtering, filtering only changes what's consumed, not what's published):")
    for n_robots in fleet_sizes:
        print(f"  n_robots={n_robots:>3}: {n_robots * BROADCAST_HZ:>6.0f} msgs/sec")


if __name__ == '__main__':
    main()
