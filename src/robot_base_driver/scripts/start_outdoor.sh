#!/usr/bin/env bash
# Host-only preflight; no GNSS serial access or clock adjustment.
set -euo pipefail
task_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
python3 "$task_root/src/wheeltec_gps_driver/src/libnmea_navsat_driver/host_utc_evidence.py" \
  --output "$task_root/.runtime_diagnostics/host_utc.json" --require-valid
exec docker exec -it Neupan bash -lc '
  source /root/miniconda3/etc/profile.d/conda.sh
  conda activate lxw
  source /opt/ros/noetic/setup.bash
  source /home/nuc/lxw/neupan_ws/devel/setup.bash
  cd /home/nuc/lxw/neupan_ws
  exec roslaunch neupan_ros full_system_run.launch outdoor_mode:=true \
    low_precision_startup:=true gps_transport_limit_sec:=4.0 gps_common_utc_verified:=true \
    gps_raw_capture_sec:=60 enable_hybridnets_overlay:=false "$@"
' bash "$@"
