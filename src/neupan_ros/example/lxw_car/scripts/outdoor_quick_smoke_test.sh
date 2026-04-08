#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <lat> <lon>"
  echo "Example: $0 23.039900 113.390100"
  exit 1
fi

LAT="$1"
LON="$2"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WAIT_INPUT="${WAIT_INPUT:-5}"
WAIT_OUTPUT="${WAIT_OUTPUT:-5}"

echo "[1/5] Checking ROS master..."
rosparam get /rosdistro >/dev/null

echo "[2/5] Checking key topics..."
for topic in /gps_goal_ll /move_base_simple/goal /gps/fix /odometry/local /odometry/global; do
  if rostopic list | grep -qx "$topic"; then
    echo "  [OK] $topic"
  else
    echo "  [WARN] Missing $topic"
  fi
done

echo "[3/5] Sending GPS goal and waiting for converted map goal..."
python3 "$SCRIPT_DIR/outdoor_gps_goal_test.py" \
  --lat "$LAT" \
  --lon "$LON" \
  --wait-input "$WAIT_INPUT" \
  --wait-output "$WAIT_OUTPUT"

echo "[4/5] Snapshot of /move_base_simple/goal"
timeout 5s rostopic echo -n 1 /move_base_simple/goal | head -n 30 || true

echo "[5/5] TF quick check (2 seconds each)"
echo "  map -> odom"
timeout 2s rosrun tf tf_echo map odom 2>&1 | head -n 12 || true
echo "  odom -> base_link"
timeout 2s rosrun tf tf_echo odom base_link 2>&1 | head -n 12 || true
echo "  base_link -> gps_link"
timeout 2s rosrun tf tf_echo base_link gps_link 2>&1 | head -n 12 || true

echo "Done."
