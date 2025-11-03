#!/usr/bin/env bash
set -eo pipefail

# --- Config ---
REQS_FILE="requirements.txt"
VENV_DIR="venv"
PY_OBSGEN_SCRIPT="tools/generate_restricted_zones.py"
PY_TARGEN_SCRIPT="tools/generate_target.py"
# ---------------

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

echo "==> 1) Ensuring base system packages"
PKGS=(python3-venv python3-yaml python3-numpy python3-packaging python3-argcomplete python3-noise python3-gz-transport13 python3-gz-msgs10)
MISSING=()
for p in "${PKGS[@]}"; do
  if ! dpkg -s "$p" >/dev/null 2>&1; then MISSING+=("$p"); fi
done
if (( ${#MISSING[@]} )); then
  echo "Installing: ${MISSING[*]}"
  sudo apt update
  sudo apt install -y "${MISSING[@]}"
else
  echo "All required system packages already present."
fi

# Ensure at least one Gazebo transport Python binding is present
if ! python3 - <<'PY'
import importlib, sys
for m in ('gz.transport13','gz.transport12','gz.transport11','gz.transport10','ignition.transport'):
    try:
        importlib.import_module(m)
        sys.exit(0)
    except Exception:
        pass
sys.exit(1)
PY
then
  echo "Installing Gazebo transport Python bindings…"
  sudo apt update
  sudo apt install -y python3-gz-transport13 || \
  sudo apt install -y python3-gz-transport12 || \
  sudo apt install -y python3-gz-transport11 || \
  sudo apt install -y python3-gz-transport10 || true
fi

# Try to install msgs (optional; we fall back if missing)
if ! python3 - <<'PY'
import importlib, sys
for m in ('gz.msgs.twist_pb2','ignition.msgs.twist_pb2'):
    try:
        importlib.import_module(m)
        sys.exit(0)
    except Exception:
        pass
sys.exit(1)
PY
then
  echo "Installing Gazebo Python msgs (optional)…"
  sudo apt update
  sudo apt install -y python3-gz-msgs11 || \
  sudo apt install -y python3-gz-msgs10 || true
fi

echo "==> 2) Creating/activating virtualenv (with system site-packages)"
if [[ ! -d "$VENV_DIR" ]]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"

echo "==> 3) Ensuring requirements.txt exists"
if [[ ! -f "$REQS_FILE" ]]; then
  cat > "$REQS_FILE" <<'EOF'
# Minimal extras (no ROS)
numpy
pyyaml
packaging
argcomplete
EOF
  echo "Created $REQS_FILE"
fi

echo "==> 4) Installing Python requirements into venv"
pip install --upgrade pip
pip install -r "$REQS_FILE"

echo "==> 5) Sanity-check: Gazebo Transport & msgs"
python - <<'PY'
import importlib, sys
ok_transport = False
for m in ("gz.transport13","gz.transport12","gz.transport11","gz.transport10","ignition.transport"):
    try:
        importlib.import_module(m); print(f"OK: import {m}"); ok_transport = True; break
    except Exception as e:
        print(f"FAIL: import {m} -> {e}")
ok_msgs = False
for m in ("gz.msgs.twist_pb2","ignition.msgs.twist_pb2"):
    try:
        importlib.import_module(m); print(f"OK: import {m}"); ok_msgs = True; break
    except Exception as e:
        print(f"WARN: import {m} -> {e}")
if not ok_transport:
    print("ERROR: No Gazebo transport Python bindings found (gz.transportXX)."); sys.exit(2)
if not ok_msgs:
    print("NOTE: Will use 'gz topic -p' fallback for publishing Twist.")
PY

if [[ "${XDG_SESSION_TYPE:-}" != "x11" ]]; then
  echo "NOTE: XDG_SESSION_TYPE=${XDG_SESSION_TYPE:-unknown}. Gazebo/keyboard UI typically works best on X11."
fi

#python3 "$PY_TARGEN_SCRIPT" # <- removed --seed 42 to randomize
./kill_ros.sh

echo "==> 6) Running controller module"
RUNS=1
for i in $(seq 1 "$RUNS"); do
  echo "==> [Run $i/$RUNS]"
  python3 -m "hydra_teleop.main" "$@"
  ./kill_ros.sh
done
