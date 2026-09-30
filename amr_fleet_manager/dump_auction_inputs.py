"""Print the exact robot/task inputs the FIFO-vs-auction benchmark used for
given trials, by replaying the benchmark's RNG sequence in the same order."""
import argparse
import numpy as np
import benchmark_fifo_vs_auction as b

p = argparse.ArgumentParser()
p.add_argument('--seed', type=int, default=42)
p.add_argument('--trials', type=int, nargs='+', default=[0])
args = p.parse_args()

rng = np.random.default_rng(args.seed)
wanted = set(args.trials)
for t in range(max(wanted) + 1):
    n_r = int(rng.integers(3, 7))
    n_t = int(rng.integers(5, 25))
    starts, tasks = b.random_batch(rng, n_r, n_t)
    if t in wanted:
        print(f"\n=== seed {args.seed}, trial {t}: {n_r} robots, {n_t} tasks ===")
        for i, s in enumerate(starts):
            print(f"  robot{i} start: ({s[0]:.2f}, {s[1]:.2f})")
        for i, tp in enumerate(tasks):
            print(f"  task{i}:        ({tp[0]:.2f}, {tp[1]:.2f})")
        for pol in ('fifo', 'auction'):
            r = b.simulate_allocation(n_r, tasks, pol, starts)
            print(f"  {pol:8s} distance={r['total_distance']:.2f}m makespan={r['makespan']:.2f}s")
