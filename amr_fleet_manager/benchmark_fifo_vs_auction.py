"""
benchmark_fifo_vs_auction.py

Compares FIFO task assignment (mission_controller.py's previous policy)
against a greedy sequential auction (distance-based bidding) for
multi-robot task allocation.

Deliberately isolated from spatial_mutex / traffic negotiation: this
benchmark measures whether WHICH robot gets WHICH task matters, not
whether robots collide getting there (that's what
benchmark_stop_and_wait_vs_hybrid.py already measured). Conflating the
two would make it impossible to tell which mechanism produced which
improvement.

Model: N robots, M pending tasks, all tasks known at t=0 (a realistic
"morning batch" scenario). Whenever robots become free, they're matched
to tasks per the active policy:
  FIFO    -- robots idle at the same moment are processed in a fixed
             order, each grabbing the oldest remaining pending task,
             regardless of distance -- matches the old
             mission_controller.py's per-robot dict-iteration behavior.
  AUCTION -- ALL robots idle at the same moment are matched against ALL
             pending tasks in one global greedy pass (always assign the
             single cheapest remaining (robot, task) pair, repeat).
             This is not a simplified approximation of
             mission_controller.run_auction() -- it is the same
             algorithm, verified to matter: a naive one-robot-at-a-time
             simulation diverges from this global-batch version in
             over 50% of random configurations.

Metrics: total fleet travel distance, and makespan.

Usage: python3 benchmark_fifo_vs_auction.py [--trials N] [--seed S]
"""

import argparse
import csv
import numpy as np


SPEED = 1.0
WAREHOUSE_SIZE = 10.0  # meters, square area tasks/robots are scattered over


def simulate_allocation(n_robots, task_positions, policy, robot_starts, speed=SPEED):
    robots = [
        {'id': i, 'pos': np.array(robot_starts[i], dtype=float), 'free_time': 0.0}
        for i in range(n_robots)
    ]
    pending = list(range(len(task_positions)))
    total_distance = 0.0
    completions = []

    while pending:
        robots.sort(key=lambda r: (r['free_time'], r['id']))
        earliest_time = robots[0]['free_time']
        tied_robots = [r for r in robots if abs(r['free_time'] - earliest_time) < 1e-9]

        if policy == 'fifo':
            for robot in sorted(tied_robots, key=lambda r: r['id']):
                if not pending:
                    break
                task_idx = pending[0]
                pending.remove(task_idx)
                task_pos = np.array(task_positions[task_idx], dtype=float)
                dist = np.linalg.norm(task_pos - robot['pos'])
                total_distance += dist
                robot['free_time'] += dist / speed
                robot['pos'] = task_pos
                completions.append(robot['free_time'])

        elif policy == 'auction':
            remaining_robot_ids = {r['id'] for r in tied_robots}
            robots_by_id = {r['id']: r for r in tied_robots}
            remaining_tasks = {ti: task_positions[ti] for ti in pending}

            while remaining_robot_ids and remaining_tasks:
                best = None
                for rid in remaining_robot_ids:
                    r = robots_by_id[rid]
                    for ti, tpos in remaining_tasks.items():
                        cost = np.linalg.norm(r['pos'] - np.array(tpos))
                        if best is None or cost < best[0]:
                            best = (cost, rid, ti)
                if best is None:
                    break
                cost, rid, ti = best
                robot = robots_by_id[rid]
                task_pos = np.array(task_positions[ti], dtype=float)
                total_distance += cost
                robot['free_time'] += cost / speed
                robot['pos'] = task_pos
                completions.append(robot['free_time'])
                remaining_robot_ids.discard(rid)
                del remaining_tasks[ti]
                pending.remove(ti)
        else:
            raise ValueError(f"unknown policy: {policy}")

    makespan = max(completions) if completions else 0.0
    return {'total_distance': total_distance, 'makespan': makespan}


def random_batch(rng, n_robots, n_tasks):
    robot_starts = [
        (float(rng.uniform(0, WAREHOUSE_SIZE)), float(rng.uniform(0, WAREHOUSE_SIZE)))
        for _ in range(n_robots)
    ]
    task_positions = [
        (float(rng.uniform(0, WAREHOUSE_SIZE)), float(rng.uniform(0, WAREHOUSE_SIZE)))
        for _ in range(n_tasks)
    ]
    return robot_starts, task_positions


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trials', type=int, default=200)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--out', type=str, default='auction_results.csv')
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    rows = []

    for trial_idx in range(args.trials):
        n_robots = int(rng.integers(3, 7))
        n_tasks = int(rng.integers(5, 25))
        robot_starts, task_positions = random_batch(rng, n_robots, n_tasks)

        for policy in ('fifo', 'auction'):
            result = simulate_allocation(n_robots, task_positions, policy, robot_starts)
            result['trial'] = trial_idx
            result['policy'] = policy
            result['n_robots'] = n_robots
            result['n_tasks'] = n_tasks
            rows.append(result)

    with open(args.out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'trial', 'policy', 'n_robots', 'n_tasks', 'total_distance', 'makespan',
        ])
        writer.writeheader()
        for r in rows:
            writer.writerow(r)

    print_summary(rows, args.trials)
    print(f"\nRaw per-trial data written to {args.out}")


def print_summary(rows, n_trials):
    for policy in ('fifo', 'auction'):
        prows = [r for r in rows if r['policy'] == policy]
        mean_dist = sum(r['total_distance'] for r in prows) / len(prows)
        mean_makespan = sum(r['makespan'] for r in prows) / len(prows)
        print(f"\n=== {policy} ({n_trials} trials) ===")
        print(f"  Mean total fleet travel distance: {mean_dist:.2f}m")
        print(f"  Mean makespan:                     {mean_makespan:.2f}s")

    fifo = {r['trial']: r for r in rows if r['policy'] == 'fifo'}
    auction = {r['trial']: r for r in rows if r['policy'] == 'auction'}

    fifo_dist = sum(r['total_distance'] for r in fifo.values())
    auction_dist = sum(r['total_distance'] for r in auction.values())
    dist_improvement = 100.0 * (fifo_dist - auction_dist) / fifo_dist

    fifo_makespan = sum(r['makespan'] for r in fifo.values())
    auction_makespan = sum(r['makespan'] for r in auction.values())
    makespan_improvement = 100.0 * (fifo_makespan - auction_makespan) / fifo_makespan

    print(f"\n=== Auction vs FIFO, same scenarios ===")
    print(f"  Total fleet travel distance reduction: {dist_improvement:+.1f}%")
    print(f"  Makespan reduction:                    {makespan_improvement:+.1f}%")


if __name__ == '__main__':
    main()
