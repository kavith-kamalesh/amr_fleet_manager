#!/usr/bin/env bash
# Usage: [SCENARIO=crossing] [LIDAR=1] [REROUTE_WAIT_SEC=2] ./scripts/repeat_smoke.sh N duration_sec label
# Runs the ROS smoke test N times, one after another, and prints one summary line per run.
N="${1:-5}"; DUR="${2:-60}"; LABEL="${3:-rep}"
if [ -d /tmp/smoke_lock ] && kill -0 "$(cat /tmp/smoke_lock/pid 2>/dev/null)" 2>/dev/null; then
  echo "ABORT: a smoke run is already active (pid $(cat /tmp/smoke_lock/pid))"; exit 75
fi
REPO="$HOME/amr_fleet_manager"; PY="$REPO/.venv/bin/python"
for i in $(seq 1 "$N"); do
  "$REPO/scripts/run_smoke_test.sh" "$DUR" "${LABEL}_$i" > /dev/null 2>&1
  LOG="$HOME/smoke_logs/${LABEL}_$i.log"; CSV="$HOME/smoke_logs/${LABEL}_$i.csv"
  planned=$(grep -c "New FMS goal" "$LOG" 2>/dev/null)
  arrived=$(grep -c "Goal reached" "$LOG" 2>/dev/null)
  rr=$(grep -ci "rerouting around" "$LOG" 2>/dev/null)
  es=$(grep -c "EMERGENCY STOP" "$LOG" 2>/dev/null)
  sep=$(env -u PYTHONPATH -u PYTHONHOME "$PY" "$REPO/scripts/smoke_odom.py" analyze --csv "$CSV" 2>/dev/null | grep "MIN SEPARATION" | sed -E 's/.*: ([0-9.]+) m.*/\1/')
  echo "run $i: planned=${planned:-0} arrived=${arrived:-0} reroutes=${rr:-0} estops=${es:-0} min_sep=${sep:-n/a}"
done
