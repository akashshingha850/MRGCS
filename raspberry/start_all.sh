#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# start_all.sh — Drone GCS onboard stack in one command (no Docker)
#
# The camera and the map come from the ZED stack (ZED SDK wrapper), which runs
# separately; this script starts what the GCS adds on top of it.
#
# Starts:
#   1. rosbridge       WebSocket ↔ ROS2, port 9090
#   2. ZED map relay   fused_cloud → gcs/map_cloud, downsampled (idle until the GCS opens the map)
#   3. MAVROS          Flight-controller bridge      (--mavros PORT to enable)
#   4. brain node      Autonomous mission planner    (--brain to enable)
#   5. barcode         Barcode / QR detector on the ZED image (--barcode to enable)
#
# Usage:
#   ./start_all.sh [OPTIONS]
#
# Options:
#        --ns     NS         Drone namespace of the MAVROS / ZED topics, e.g. /drone_1  (default: none)
#        --mavros FCTL_PORT  Enable MAVROS  (e.g. /dev/ttyAMA10)
#        --brain             Enable brain / mission-planner node
#        --barcode           Enable barcode detector
#        --ros-distro NAME   ROS2 distro name  (default: jazzy)
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Defaults ──────────────────────────────────────────────────────────────────
DRONE_NS=""
ENABLE_MAVROS=0
MAVROS_PORT="/dev/ttyAMA10"
ENABLE_BRAIN=0
ENABLE_BARCODE=0
ROS_DISTRO_OVERRIDE="${ROS_DISTRO:-jazzy}"

# ── Argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ns)            DRONE_NS="$2";                           shift 2 ;;
    --mavros)        ENABLE_MAVROS=1; MAVROS_PORT="$2";       shift 2 ;;
    --brain)         ENABLE_BRAIN=1;                          shift   ;;
    --barcode)       ENABLE_BARCODE=1;                        shift   ;;
    --ros-distro)    ROS_DISTRO_OVERRIDE="$2";                shift 2 ;;
    *) echo "[GCS] Unknown option: $1"; shift ;;
  esac
done

# ── Source ROS2 ───────────────────────────────────────────────────────────────
set +u
source "/opt/ros/$ROS_DISTRO_OVERRIDE/setup.bash"
source "$HOME/rosbridge_ws/install/setup.bash" 2>/dev/null || true
set -u

NS_ARGS=()
[[ -n "$DRONE_NS" ]] && NS_ARGS=(--ros-args -r "__ns:=$DRONE_NS")

# ── Logging ───────────────────────────────────────────────────────────────────
LOG_DIR="$HOME/gcs_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

log()  { echo "[GCS $(date +%H:%M:%S)] $*"; }

log "Logging to $LOG_DIR"

# ── PID tracking & cleanup ───────────────────────────────────────────────────
declare -A PIDS

cleanup() {
  log "Shutting down all processes..."
  for name in "${!PIDS[@]}"; do
    pid="${PIDS[$name]}"
    if kill -0 "$pid" 2>/dev/null; then
      log "  Stopping $name (PID $pid)"
      kill "$pid" 2>/dev/null
    fi
  done
  wait 2>/dev/null || true
  log "All stopped. Logs at $LOG_DIR"
}
trap cleanup EXIT INT TERM

# ─────────────────────────────────────────────────────────────────────────────
# 1. rosbridge — WebSocket ↔ ROS2 on port 9090
# ─────────────────────────────────────────────────────────────────────────────
log "Starting rosbridge on :9090 ..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  > "$LOG_DIR/rosbridge.log" 2>&1 &
PIDS[rosbridge]=$!
sleep 2   # give rosbridge time to bind before other nodes publish

# ─────────────────────────────────────────────────────────────────────────────
# 2. ZED map relay — downsampled spatial map for the GCS SLAM tab
# ─────────────────────────────────────────────────────────────────────────────
log "Starting ZED map relay (ns ${DRONE_NS:-/})"
python3 "$SCRIPT_DIR/zed_map_relay.py" "${NS_ARGS[@]}" \
  > "$LOG_DIR/map_relay.log" 2>&1 &
PIDS[map_relay]=$!

log "Starting ZED image relay (depth + confidence for the GCS Image tab)"
python3 "$SCRIPT_DIR/zed_image_relay.py" "${NS_ARGS[@]}" \
  > "$LOG_DIR/image_relay.log" 2>&1 &
PIDS[image_relay]=$!

# ─────────────────────────────────────────────────────────────────────────────
# 3. MAVROS — flight-controller bridge (disabled unless --mavros PORT given)
# ─────────────────────────────────────────────────────────────────────────────
if [[ "$ENABLE_MAVROS" == "1" ]]; then
  log "Starting MAVROS on $MAVROS_PORT"
  MAV_PORT="$MAVROS_PORT" bash "$SCRIPT_DIR/mavros_launch.sh" \
    > "$LOG_DIR/mavros.log" 2>&1 &
  PIDS[mavros]=$!
fi

# ─────────────────────────────────────────────────────────────────────────────
# 4. Brain node — autonomous mission planner (disabled unless --brain given)
# ─────────────────────────────────────────────────────────────────────────────
if [[ "$ENABLE_BRAIN" == "1" ]]; then
  log "Starting brain node  (DB: ~/brain_data.db)"
  python3 "$SCRIPT_DIR/brain_node.py" "${NS_ARGS[@]}" \
    > "$LOG_DIR/brain.log" 2>&1 &
  PIDS[brain]=$!
fi

# ─────────────────────────────────────────────────────────────────────────────
# 5. Barcode detector — on the ZED left image (disabled unless --barcode given)
# ─────────────────────────────────────────────────────────────────────────────
if [[ "$ENABLE_BARCODE" == "1" ]]; then
  log "Starting barcode detector"
  python3 "$SCRIPT_DIR/barcode_detector.py" "${NS_ARGS[@]}" \
    > "$LOG_DIR/barcode.log" 2>&1 &
  PIDS[barcode]=$!
fi

# ─────────────────────────────────────────────────────────────────────────────
# Status banner
# ─────────────────────────────────────────────────────────────────────────────
GCS_IP=$(hostname -I | awk '{print $1}')
echo ""
log "══════════════════════════════════════════════════"
log "  Drone GCS — all systems running"
log "  rosbridge  ws://$GCS_IP:9090"
log "  Logs       $LOG_DIR"
log "  Processes:"
for name in "${!PIDS[@]}"; do
  log "    $name  PID ${PIDS[$name]}"
done
log "  Press Ctrl+C to stop everything."
log "══════════════════════════════════════════════════"
echo ""

wait
