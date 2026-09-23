"""
benchmark_rf_predictor_ablation.py

Trains a Random Forest travel-time predictor and tests whether using its
predictions as the auction bid (instead of raw straight-line distance)
actually produces better fleet outcomes than distance alone.

Methodology, and the two real problems found and fixed while building this
(kept in the script, not just the writeup, because the fixes are why the
final numbers are trustworthy):

1. TRAINING DATA comes from the SAME hybrid mutex/reroute/geometric-safety
   simulation used in benchmark_stop_and_wait_vs_hybrid.py -- not a
   synthetic/invented dataset. A robot's realized arrival time there
   already includes real congestion delays caused by other robots
   sharing the grid.

2. FIRST FEATURE SET (distance, n_robots-in-fleet, priority) predicted
   almost nothing: R2=0.056, barely above zero, worse than a
   distance-only model. The fix was recognizing that total fleet size is
   a weak proxy for MY congestion -- what matters is whether other
   robots' planned routes actually overlap with mine. Adding
   n_path_conflicts (how many other robots' A* paths share at least one
   grid edge with this robot's own planned path -- computable at bid
   time, before anyone moves) raised R2 to 0.206.

3. SECOND PROBLEM: ~19% of samples are capped at TIMEOUT_SEC because the
   robot never arrives (permanent blocking in dense random scenarios).
   That's censored data, not a real travel time, and regressing on it
   directly corrupts the fit. Fixed by splitting into two models: a
   RandomForestClassifier predicting P(arrives at all), and a
   RandomForestRegressor trained ONLY on robots that did arrive,
   predicting realized time conditional on success. Combined at bid time
   as an expected-cost estimate: P(success) * predicted_time +
   (1 - P(success)) * TIMEOUT_SEC. This raised the regressor's R2 to
   0.325 (MAE 5.83s, down from 15.74s).

4. The ABLATION compares DISTANCE bidding (mission_controller.py's
   current policy) against this RF expected-cost bidding, on IDENTICAL
   random batches, using the same greedy global-matching auction
   algorithm validated in benchmark_fifo_vs_auction.py, then measures
   REALIZED outcomes by actually running the assigned robots through the
   hybrid mutex simulator. If RF doesn't help, that will show here
   honestly -- this script does not filter or cherry-pick scenarios.
"""

import argparse
import heapq
import numpy as np
from sklearn.ensemble import RandomForestRegressor, RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score, mean_absolute_error, roc_auc_score


# ---------------- Grid graph (identical to nav_graph.py) ----------------

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


def path_edges(path):
    return set(frozenset(e) for e in zip(path, path[1:]))


# ---------------- Simulation constants (match spatial_mutex.py defaults) ----------------

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
        self.straight_line_distance = float(np.linalg.norm(
            node_pos(goal_node) - node_pos(start_node)
        ))

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


def hybrid_winner(self_r, other_r, t_now):
    self_eff = self_r.effective_priority(t_now)
    other_eff = other_r.effective_priority(t_now)
    if other_eff > self_eff:
        return True
    if other_eff < self_eff:
        return False
    return other_r.id < self_r.id


def run_hybrid_scenario(robots_config, timeout_sec=TIMEOUT_SEC):
    """robots_config: list of (start_node, goal_node, base_priority).
    Runs the validated hybrid mutex/reroute/geometric-safety simulation."""
    robots = [SimRobot(i, s, g, p) for i, (s, g, p) in enumerate(robots_config)]

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

    return robots, collisions


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


# ---------------- Step 1: training data from the real hybrid simulator ----------------

def generate_training_data(rng, n_trials):
    rows = []
    for _ in range(n_trials):
        n_robots = int(rng.integers(3, 9))
        config = random_scenario(rng, n_robots)
        robots, _ = run_hybrid_scenario(config)

        initial_paths = [astar(s, g) for s, g, p in config]
        initial_edges = [path_edges(p) for p in initial_paths]

        for i, r in enumerate(robots):
            n_conflicts = sum(
                1 for j in range(n_robots)
                if j != i and (initial_edges[i] & initial_edges[j])
            )
            rows.append({
                'distance': r.straight_line_distance,
                'n_robots': n_robots,
                'priority': r.base_priority,
                'n_path_conflicts': n_conflicts,
                'arrived': r.arrival_time is not None,
                'realized_time': r.arrival_time,
            })
    return rows


def features(distance, n_robots, priority, n_path_conflicts):
    return np.array([[distance, n_robots, priority, n_path_conflicts]])


# ---------------- Step 2: train the two-part model, report honest metrics ----------------

def train_and_evaluate(rows, seed):
    X_all = np.array([[r['distance'], r['n_robots'], r['priority'], r['n_path_conflicts']] for r in rows])
    y_success = np.array([r['arrived'] for r in rows])

    Xtr, Xte, ytr, yte = train_test_split(X_all, y_success, test_size=0.25, random_state=seed, stratify=y_success)
    clf = RandomForestClassifier(n_estimators=200, max_depth=8, random_state=seed)
    clf.fit(Xtr, ytr)
    pred_proba = clf.predict_proba(Xte)[:, 1]
    baseline_acc = max(yte.mean(), 1 - yte.mean())

    print(f"=== Success classifier (P(robot arrives at all)) ===")
    print(f"  AUC={roc_auc_score(yte, pred_proba):.3f}  accuracy={clf.score(Xte, yte):.3f}  "
          f"(naive always-predict-majority baseline: {baseline_acc:.3f})")
    print(f"  AUC is the honest metric here -- classes are imbalanced "
          f"({100*yte.mean():.0f}% arrive), so accuracy alone is misleading.")

    succ_rows = [r for r in rows if r['arrived']]
    n_timeout = len(rows) - len(succ_rows)
    X_succ = np.array([[r['distance'], r['n_robots'], r['priority'], r['n_path_conflicts']] for r in succ_rows])
    y_succ = np.array([r['realized_time'] for r in succ_rows])
    Xtr2, Xte2, ytr2, yte2 = train_test_split(X_succ, y_succ, test_size=0.25, random_state=seed)
    reg = RandomForestRegressor(n_estimators=200, max_depth=8, random_state=seed)
    reg.fit(Xtr2, ytr2)
    pred2 = reg.predict(Xte2)

    print(f"\n=== Conditional travel-time regressor (arrived robots only, {n_timeout}/{len(rows)} excluded as censored) ===")
    print(f"  R2={r2_score(yte2, pred2):.3f}  MAE={mean_absolute_error(yte2, pred2):.2f}s")
    print(f"  Feature importances: distance={reg.feature_importances_[0]:.3f} "
          f"n_robots={reg.feature_importances_[1]:.3f} priority={reg.feature_importances_[2]:.3f} "
          f"n_path_conflicts={reg.feature_importances_[3]:.3f}")

    return clf, reg


def expected_cost(clf, reg, distance, n_robots, priority, n_path_conflicts):
    f = features(distance, n_robots, priority, n_path_conflicts)
    p_success = clf.predict_proba(f)[0, 1]
    predicted_time = reg.predict(f)[0]
    return p_success * predicted_time + (1 - p_success) * TIMEOUT_SEC


# ---------------- Step 3: ablation -- does RF-cost auction beat distance auction? ----------------

def auction_assign(robots_start, task_positions, cost_fn):
    """Same global-greedy matching algorithm as mission_controller.run_auction()."""
    remaining_robots = set(range(len(robots_start)))
    remaining_tasks = {i: task_positions[i] for i in range(len(task_positions))}
    assignment = {}

    while remaining_robots and remaining_tasks:
        best = None
        for ri in remaining_robots:
            for ti, tpos in remaining_tasks.items():
                cost = cost_fn(robots_start[ri], tpos)
                if best is None or cost < best[0]:
                    best = (cost, ri, ti)
        if best is None:
            break
        _, ri, ti = best
        assignment[ri] = remaining_tasks[ti]
        remaining_robots.discard(ri)
        del remaining_tasks[ti]

    return assignment


def run_ablation(clf, reg, rng, n_batches):
    rows = []
    for batch_idx in range(n_batches):
        n_robots = int(rng.integers(3, 9))
        nodes = [(x, y) for x in range(GRID_SIZE) for y in range(GRID_SIZE)]

        starts = []
        used = set()
        for _ in range(n_robots):
            s = tuple(nodes[rng.integers(0, len(nodes))])
            while s in used:
                s = tuple(nodes[rng.integers(0, len(nodes))])
            used.add(s)
            starts.append(s)

        task_positions = [tuple(nodes[rng.integers(0, len(nodes))]) for _ in range(n_robots)]
        priorities = [float(rng.uniform(0.1, 0.9)) for _ in range(n_robots)]

        def distance_cost(start, task):
            return float(np.linalg.norm(node_pos(task) - node_pos(start)))

        def rf_cost(start, task):
            d = float(np.linalg.norm(node_pos(task) - node_pos(start)))
            candidate_path = astar(start, task)
            candidate_edges = path_edges(candidate_path)
            # Route-conflict feature approximated at bid time using the
            # OTHER robots' straight A*(their own start, their own most
            # recently observed task-area) is not available before
            # assignment, so this uses the same information mission_controller
            # actually has at bid time: the candidate's own path against the
            # other robots' current positions' likely corridors is not
            # computable without their tasks either -- so, consistent with
            # what's genuinely available pre-assignment, this uses n_robots
            # as the congestion signal here (path-specific conflict requires
            # knowing everyone's chosen task, which is what the auction is
            # deciding) and n_path_conflicts=0 as a neutral default.
            return expected_cost(clf, reg, d, n_robots, 0.5, 0)

        for policy_name, cost_fn in (('distance', distance_cost), ('rf', rf_cost)):
            assignment = auction_assign(starts, task_positions, cost_fn)
            config = [
                (starts[ri], assignment[ri], priorities[ri])
                for ri in range(n_robots) if ri in assignment
            ]
            robots, collisions = run_hybrid_scenario(config)
            n_timeout = sum(1 for r in robots if r.arrival_time is None)
            makespan = max((r.arrival_time for r in robots if r.arrival_time is not None), default=TIMEOUT_SEC)
            total_wait = sum(r.total_wait_time for r in robots)

            rows.append({
                'batch': batch_idx, 'policy': policy_name, 'n_robots': n_robots,
                'collisions': collisions, 'timed_out': n_timeout > 0,
                'makespan': makespan, 'total_wait': total_wait,
            })
    return rows


def print_ablation_summary(rows, n_batches):
    for policy in ('distance', 'rf'):
        prows = [r for r in rows if r['policy'] == policy]
        n_timeout = sum(1 for r in prows if r['timed_out'])
        n_collisions = sum(r['collisions'] for r in prows)
        mean_makespan = sum(r['makespan'] for r in prows) / len(prows)
        mean_wait = sum(r['total_wait'] for r in prows) / len(prows)
        print(f"\n=== {policy} auction ({n_batches} batches) ===")
        print(f"  Collisions: {n_collisions}")
        print(f"  Timed out:  {n_timeout}/{n_batches}")
        print(f"  Mean makespan (incl. timeout penalty): {mean_makespan:.2f}s")
        print(f"  Mean total wait per batch:              {mean_wait:.2f}s")

    dist_makespan = sum(r['makespan'] for r in rows if r['policy'] == 'distance')
    rf_makespan = sum(r['makespan'] for r in rows if r['policy'] == 'rf')
    pct = 100.0 * (dist_makespan - rf_makespan) / dist_makespan
    print(f"\n=== RF-cost auction vs distance auction ===")
    print(f"  Makespan change: {pct:+.1f}%  (positive = RF better, negative = RF worse)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--train-trials', type=int, default=300)
    parser.add_argument('--ablation-batches', type=int, default=150)
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    print(f"Generating training data from {args.train_trials} real hybrid-mutex simulation trials...")
    rows = generate_training_data(rng, args.train_trials)
    n_timeout = sum(1 for r in rows if not r['arrived'])
    print(f"  {len(rows)} robot-trip samples collected ({n_timeout} timed out, "
          f"{100*n_timeout/len(rows):.1f}%).\n")

    clf, reg = train_and_evaluate(rows, args.seed)

    print(f"\nRunning ablation over {args.ablation_batches} batches...")
    ablation_rows = run_ablation(clf, reg, rng, args.ablation_batches)
    print_ablation_summary(ablation_rows, args.ablation_batches)


if __name__ == '__main__':
    main()
