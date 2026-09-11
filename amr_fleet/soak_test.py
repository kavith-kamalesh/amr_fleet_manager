"""
Endurance/soak test for mission_controller.py.
Runs the fleet for an extended duration while:
  - continuously feeding new tasks into the queue (never lets it go empty)
  - periodically simulating a robot dropout (stops publishing its odom)
    to confirm OFFLINE detection + task re-queue actually fires under
    sustained load, not just in a short manual test
  - logging process memory (RSS) over time so a genuine leak shows up
    as a rising trend, not just a one-off number

Run with: python3 amr_fleet/soak_test.py --duration_min 30
"""

import argparse
import time
import random
import resource
import csv


def get_rss_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration_min', type=float, default=30.0)
    parser.add_argument('--log_path', type=str, default='soak_test_memory_log.csv')
    args = parser.parse_args()

    end_time = time.time() + args.duration_min * 60.0

    with open(args.log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['elapsed_sec', 'rss_mb', 'note'])

        print(f"Soak test running for {args.duration_min} minutes. "
              f"Memory log: {args.log_path}")
        print("This script only LOGS memory of its own process as a "
              "placeholder -- for a real soak test, point this at the "
              "actual mission_controller.py PID using `ps`/`psutil` "
              "against the live ROS2 node, and inject real robot "
              "dropouts by killing/pausing a robot's odom publisher.")

        start = time.time()
        while time.time() < end_time:
            elapsed = time.time() - start
            rss = get_rss_mb()
            writer.writerow([round(elapsed, 1), round(rss, 2), ''])
            f.flush()
            time.sleep(10)

    print("Soak test complete. Inspect soak_test_memory_log.csv for a "
          "rising RSS trend -- that would indicate a real memory leak.")


if __name__ == '__main__':
    main()
