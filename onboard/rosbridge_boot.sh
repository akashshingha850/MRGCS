#!/usr/bin/env bash
# rosbridge_boot.sh — started by systemd (or the Docker image) on every boot.
# Brings up rosbridge (port 9090), the ZED map relay (idle until the GCS opens the
# map), the GCS web app (http://<host>:$GCS_WEB_PORT, when set) and the GCS
# control node (receives dashboard commands).
# MAVROS, brain and barcode are started on demand from the app.
#
# DRONE_NS (e.g. /drone_1) puts the relay — and, through gcs_control, brain and
# barcode — in the namespace of the drone's MAVROS / ZED topics.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source "$HOME/rosbridge_ws/install/setup.bash" 2>/dev/null || true

NS_ARGS=()
[[ -n "${DRONE_NS:-}" ]] && NS_ARGS=(--ros-args -r "__ns:=$DRONE_NS")

echo "[boot] Starting rosbridge on :9090 ..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
BRIDGE_PID=$!

echo "[boot] Starting ZED map relay (ns ${DRONE_NS:-/}) ..."
python3 "$SCRIPT_DIR/zed_map_relay.py" "${NS_ARGS[@]}" &
RELAY_PID=$!

echo "[boot] Starting ZED image relay (depth + confidence JPEGs, ns ${DRONE_NS:-/}) ..."
python3 "$SCRIPT_DIR/zed_image_relay.py" "${NS_ARGS[@]}" &
IMG_RELAY_PID=$!

# Against a sim/drone MAVROS that is not ours (DRONE_NS set): ask PX4 for the motor outputs the dashboard knobs show
if [[ -n "${DRONE_NS:-}" ]]; then
  echo "[boot] Requesting motor-output streams from the FCU (ns $DRONE_NS) ..."
  MAVROS_STREAMS=rc python3 "$SCRIPT_DIR/mavros_streams.py" "${NS_ARGS[@]}" &
  STREAMS_PID=$!
fi

# The GCS app is plain static files: any browser can run it, no install
WEB_PID=""
if [[ -n "${GCS_WEB_PORT:-}" ]]; then
  echo "[boot] Serving the GCS web app on :$GCS_WEB_PORT ..."
  python3 -m http.server "$GCS_WEB_PORT" --directory "$SCRIPT_DIR/../renderer" >/dev/null 2>&1 &
  WEB_PID=$!
fi

# Sensors tab: live sources, the analyse flight, which source EKF2 should fuse. On sim time against the sim (USE_SIM_TIME=true): its turn rate
# and hold times are then the drone's, not the slower wall clock's
echo "[boot] Starting sensor check node (ns ${DRONE_NS:-/}) ..."
python3 "$SCRIPT_DIR/sensor_check.py" --ros-args ${DRONE_NS:+-r "__ns:=$DRONE_NS"} -p use_sim_time:=${USE_SIM_TIME:-false} &
SENSORS_PID=$!

# Stop them all when this script exits (systemd stop / reboot / crash)
trap "kill $BRIDGE_PID $RELAY_PID $WEB_PID $STREAMS_PID $IMG_RELAY_PID $SENSORS_PID 2>/dev/null; wait" EXIT INT TERM

# Give rosbridge a moment to bind before the control node tries to publish
sleep 3

echo "[boot] Starting GCS control node ..."
python3 "$SCRIPT_DIR/gcs_control.py"
