#!/usr/bin/env bash
# mavros_launch.sh — MAVROS (ArduCopter) on the Pixhawk serial link.
#   1. Pick the serial port (RPi5 RP1 UART aware) unless MAV_PORT is set
#   2. Start MAVROS with the ArduPilot plugin set (apm.launch)
#   3. Start mavros_streams.py — requests telemetry stream rates on every
#      (re)connect, since ArduCopter does not stream on its own over a
#      companion-computer UART unless SRx_ params are configured
#
# Called by gcs_control.py when the MAVROS button is pressed in the dashboard,
# and by start_all.sh --mavros.
#
# Environment overrides:
#   MAV_PORT=/dev/ttyAMA10   serial port   (default: auto-detect)
#   MAV_BAUD=57600           baud rate     (default: 57600)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash

BAUD="${MAV_BAUD:-57600}"

# ── 1. Serial port ────────────────────────────────────────────────────────────
# ttyAMA10 intentionally precedes ttyAMA0 — on RPi5, ttyAMA0 may be mapped to
# the Bluetooth chip (RP1), not to the GPIO 14/15 UART, and opens silently
# with no MAVLink data.
if [[ -n "$MAV_PORT" ]]; then
    PORT="$MAV_PORT"
else
    PORT="/dev/serial0"
    for p in /dev/serial0 /dev/ttyAMA10 /dev/ttyAMA0 /dev/ttyUSB0 /dev/ttyACM0; do
        if [[ -e "$p" ]]; then PORT="$p"; break; fi
    done
fi
# MAVROS opens the port as given; resolve the serial0 symlink so its logs show
# the real device
PORT="$(readlink -f "$PORT")"

if [[ ! -r "$PORT" || ! -w "$PORT" ]]; then
    echo "[mavros] WARNING: no read/write access to $PORT"
    echo "[mavros]   sudo usermod -aG dialout \$USER   (then log out/in)"
fi

# ── 2. MAVROS ─────────────────────────────────────────────────────────────────
# gcs_url is left at apm.launch's empty default: `gcs_url:=` (empty value) is rejected by ros2 launch
echo "[mavros] Starting MAVROS — fcu_url=$PORT:$BAUD"
ros2 launch mavros apm.launch fcu_url:="$PORT:$BAUD" &
MAVROS_PID=$!

# ── 3. Stream-rate requester ──────────────────────────────────────────────────
python3 "$SCRIPT_DIR/mavros_streams.py" &
STREAMS_PID=$!

# Stop both children when this script is stopped (gcs_control / start_all.sh)
trap "kill $STREAMS_PID $MAVROS_PID 2>/dev/null; wait" EXIT INT TERM

wait $MAVROS_PID
