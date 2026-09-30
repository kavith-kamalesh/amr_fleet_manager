#!/usr/bin/env bash
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"

PRUNE=( -not -path './.venv/*' -not -path './build/*' -not -path './install/*' -not -path './log/*' )

echo "[1/3] syntax check (catches truncated files)"
find . -name '*.py' "${PRUNE[@]}" -print0 | xargs -0 python -m py_compile

echo "[2/3] pytest"
python -m pytest tests -q

echo "[3/3] duplicate module names (warning only)"
DUPS=$(find . -name '*.py' "${PRUNE[@]}" -not -name '__init__.py' -not -name 'setup.py' -exec basename {} \; | sort | uniq -d)
if [ -n "$DUPS" ]; then
  echo "WARNING: same filename exists in more than one place:"
  echo "$DUPS"
fi
echo "[4] analysis scripts run against committed results"
python scripts/summarize_benchmarks.py > /dev/null
python scripts/analyze_timeout_sensitivity.py > /dev/null
echo "preflight OK"
