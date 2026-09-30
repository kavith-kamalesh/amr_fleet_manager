#!/usr/bin/env bash
# Regenerates the stop-and-wait vs hybrid headline numbers into results/.
# expiry5s        = deployed logic (mirrors waypoint_nav_node.BLOCK_EXPIRY_SEC)
# legacy_noexpiry = pre-736f1c2 behavior, kept so the comparison stays honest
set -euo pipefail
REPO="$(git rev-parse --show-toplevel)"
cd "$REPO"
BENCH=amr_fleet_manager/benchmark_stop_and_wait_vs_hybrid.py
mkdir -p results
for pair in expiry5s:5.0 legacy_noexpiry:inf; do
  name="${pair%%:*}"; val="${pair##*:}"
  tmp="$(mktemp -d)"
  (cd "$tmp" && BLOCK_EXPIRY_SEC="$val" PYTHONPATH="$REPO" python "$REPO/$BENCH" > "$REPO/results/swh_${name}.stdout.txt")
  cp "$tmp/benchmark_results.csv" "results/swh_${name}.csv"
  rm -rf "$tmp"
  echo "=== $name ==="
  python scripts/summarize_benchmarks.py "results/swh_${name}.csv" | sed -n 1,11p
  python scripts/analyze_timeout_sensitivity.py "results/swh_${name}.csv" | sed -n 1,9p
done

# Row 3: 5s expiry, but proximity (physical) blocks do NOT trigger a reroute.
# Matches spatial_mutex.py/waypoint_nav_node.py as of this commit: a physical block
# causes an e-stop; only a reservation conflict escalates to REROUTE_REQUESTED.
tmp="$(mktemp -d)"
(cd "$tmp" && BLOCK_EXPIRY_SEC=5.0 REROUTE_ON_GEOMETRIC=0 PYTHONPATH="$REPO" python "$REPO/$BENCH" > "$REPO/results/swh_expiry5s_noGeoReroute.stdout.txt")
cp "$tmp/benchmark_results.csv" results/swh_expiry5s_noGeoReroute.csv
rm -rf "$tmp"
echo "=== expiry5s_noGeoReroute ==="
python scripts/summarize_benchmarks.py results/swh_expiry5s_noGeoReroute.csv | sed -n 1,11p
python scripts/analyze_timeout_sensitivity.py results/swh_expiry5s_noGeoReroute.csv | sed -n 1,9p
