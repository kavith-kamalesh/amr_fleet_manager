"""
Benchmark: Hybrid (A* + space-time reservation + dynamic rerouting)
vs. Stop-and-Wait (naive full-halt baseline, no rerouting).

Directly measures the SIH success criterion: "minimum 20% reduction
in total task completion time compared to traditional stop-and-wait
methods when handling overlapping paths."

Both modes share the exact same grid, robots, and randomized task
batches per trial -- the only thing that differs is the conflict
policy. Reports makespan (time until all robots finish all assigned
tasks) for each mode, the measured % improvement, and a collision
count sanity check for both.

Run: python3 benchmark_stop_and_wait_vs_hybrid.py --trials 20 --tasks_per_robot 6
"""

import argparse
import heapq
import random

GRID_SIZE = 6
CELL = 1.0
SPEED = 1.0
DT = 0.05
ROBOT_RADIUS = 0.3
REROUTE_WAIT_THRESHOLD = 1.5
RESERVATION_BUFFER = 0.3
MAX_SIM_SECONDS = 600.0  # safety cap so a stuck run doesn't hang forever


def neighbors(n):
    x, y = n
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            yield (nx, ny)


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, blocked_edges=frozenset()):
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


class Robot:
    def __init__(self, rid, start_node, priority):
        self.id = rid
        self.priority = priority
        self.pos_node = start_node
        self.path = None
        self.path_idx = 0
        self.edge_progress = 0.0
        self.state = "IDLE"
        self.wait_time = 0.0
        self.tasks_remaining = []
        self.tasks_completed = 0

    def current_edge(self):
        if self.path is None or self.path_idx >= len(self.path) - 1:
            return None
        return (self.path[self.path_idx], self.path[self.path_idx + 1])

    def edge_time_window(self, t_now):
        remaining_frac = 1.0 - self.edge_progress
        edge_duration = CELL / SPEED
        t_start = t_now - self.edge_progress * edge_duration
        t_end = t_start + edge_duration
        return (t_start - RESERVATION_BUFFER, t_end + RESERVATION_BUFFER)


def windows_conflict(w1, w2):
    return not (w1[1] < w2[0] or w2[1] < w1[0])


def make_random_tasks(n_tasks, rng):
    return [(rng.randrange(GRID_SIZE), rng.randrange(GRID_SIZE)) for _ in range(n_tasks)]


def run_trial(mode, n_robots, tasks_per_robot, seed):
    """mode: 'hybrid' or 'stop_and_wait'. Returns (makespan_seconds, collisions)."""
    rng = random.Random(seed)

    starts = [(0, 0), (GRID_SIZE - 1, 0), (0, GRID_SIZE - 1), (GRID_SIZE - 1, GRID_SIZE - 1), (GRID_SIZE // 2, 0)]
    priorities = [0.9, 0.6, 0.3, 0.7, 0.5]

    robots = []
    for i in range(n_robots):
        r = Robot(i, starts[i % len(starts)], priorities[i % len(priorities)])
        r.tasks_remaining = make_random_tasks(tasks_per_robot, rng)
        robots.append(r)

    t = 0.0
    collisions = 0
    total_tasks = n_robots * tasks_per_robot

    while t < MAX_SIM_SECONDS:
        t += DT

        for r in robots:
            if r.path is None and r.tasks_remaining:
                goal = r.tasks_remaining.pop(0)
                r.path = astar(r.pos_node, goal)
                r.path_idx = 0
                r.edge_progress = 0.0
                if r.path is None:
                    r.tasks_completed += 1  # unreachable goal counted as skipped, not stuck
                    continue

            edge = r.current_edge()
            if edge is None:
                continue

            my_window = r.edge_time_window(t)
            my_nodes = set(edge)

            blocked = False
            for other in robots:
                if other.id == r.id:
                    continue
                other_edge = other.current_edge()
                if other_edge is None:
                    continue
                other_nodes = set(other_edge)
                if not (my_nodes & other_nodes):
                    continue

                if mode == "stop_and_wait":
                    # Naive baseline: ANY shared node with ANY other robot
                    # currently on that edge = full stop, no time-window
                    # nuance, no rerouting, ever.
                    if other.priority >= r.priority:
                        blocked = True
                        break
                else:
                    other_window = other.edge_time_window(t)
                    if windows_conflict(my_window, other_window) and other.priority >= r.priority:
                        blocked = True
                        break

            if blocked:
                r.wait_time += DT
                if mode == "hybrid" and r.wait_time > REROUTE_WAIT_THRESHOLD:
                    goal = r.path[-1]
                    new_path = astar(r.pos_node, goal, blocked_edges=frozenset({edge}))
                    if new_path:
                        r.path = new_path
                        r.path_idx = 0
                        r.edge_progress = 0.0
                        r.wait_time = 0.0
                continue

            r.wait_time = 0.0
            r.edge_progress += (SPEED * DT) / CELL
            if r.edge_progress >= 1.0:
                r.pos_node = edge[1]
                r.path_idx += 1
                r.edge_progress = 0.0
                if r.path_idx >= len(r.path) - 1:
                    r.path = None
                    r.tasks_completed += 1

        # Collision check (should be 0 for both modes if logic is correct)
        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                if robots[i].pos_node == robots[j].pos_node:
                    collisions += 1

        if all(r.tasks_completed >= tasks_per_robot for r in robots):
            return round(t, 2), collisions

    return round(MAX_SIM_SECONDS, 2), collisions  # hit cap -- treat as worst case


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--n_robots', type=int, default=3)
    parser.add_argument('--tasks_per_robot', type=int, default=6)
    args = parser.parse_args()

    hybrid_times, saw_times = [], []
    hybrid_collisions, saw_collisions = 0, 0

    print(f"Running {args.trials} trials | {args.n_robots} robots | "
          f"{args.tasks_per_robot} tasks/robot | grid {GRID_SIZE}x{GRID_SIZE}\n")

    for trial in range(args.trials):
        seed = 1000 + trial

        hybrid_makespan, hc = run_trial("hybrid", args.n_robots, args.tasks_per_robot, seed)
        saw_makespan, sc = run_trial("stop_and_wait", args.n_robots, args.tasks_per_robot, seed)

        hybrid_times.append(hybrid_makespan)
        saw_times.append(saw_makespan)
        hybrid_collisions += hc
        saw_collisions += sc

        improvement = (saw_makespan - hybrid_makespan) / saw_makespan * 100 if saw_makespan > 0 else 0
        print(f"Trial {trial+1:2d} | stop-and-wait: {saw_makespan:6.2f}s | "
              f"hybrid: {hybrid_makespan:6.2f}s | improvement: {improvement:5.1f}%")

    avg_hybrid = sum(hybrid_times) / len(hybrid_times)
    avg_saw = sum(saw_times) / len(saw_times)
    avg_improvement = (avg_saw - avg_hybrid) / avg_saw * 100 if avg_saw > 0 else 0

    print("\n" + "=" * 60)
    print(f"AVERAGE stop-and-wait makespan: {avg_saw:.2f}s")
    print(f"AVERAGE hybrid makespan:        {avg_hybrid:.2f}s")
    print(f"AVERAGE improvement:            {avg_improvement:.1f}%")
    print(f"Total collisions -- stop-and-wait: {saw_collisions} | hybrid: {hybrid_collisions}")
    print("=" * 60)

    target = 20.0
    if avg_improvement >= target:
        print(f"\nRESULT: PASSES success criterion (>= {target}% improvement).")
    else:
        print(f"\nRESULT: DOES NOT YET MEET the {target}% target with current parameters. "
              f"Consider tuning REROUTE_WAIT_THRESHOLD, grid size, or task density and re-running.")


if __name__ == '__main__':
    main()
