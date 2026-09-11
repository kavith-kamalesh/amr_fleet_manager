import argparse
import time
import csv
import os
import psutil

def get_rss_mb():
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 * 1024)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration_min', type=float, default=30.0)
    parser.add_argument('--log_path', type=str, default='soak_test_memory_log.csv')
    args = parser.parse_args()
    end_time = time.time() + args.duration_min * 60.0
    
    print(f"Soak test running for {args.duration_min} minutes. Log: {args.log_path}")
    
    with open(args.log_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['elapsed_sec', 'rss_mb', 'note'])
        start = time.time()
        while time.time() < end_time:
            elapsed = time.time() - start
            rss = get_rss_mb()
            writer.writerow([round(elapsed, 1), round(rss, 2), ''])
            f.flush()
            time.sleep(10)
            
    print("Soak test complete.")

if __name__ == '__main__':
    main()
