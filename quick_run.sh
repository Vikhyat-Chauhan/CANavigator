#!/usr/bin/env bash
set -eo pipefail

# --- Config ---
VENV_DIR="venv"

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

./kill_ros.sh
rm -rf logs

# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

echo "==> 6) Running controller module"
RUNS=1
for i in $(seq 1 "$RUNS"); do
  echo "==> [Run $i/$RUNS]"
  python3 -m "hydra_teleop.main" "$@"
  ./kill_ros.sh
done