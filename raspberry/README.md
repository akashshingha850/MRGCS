# raspberry/ — Onboard Software Stack

All Python nodes and shell scripts that run on the drone's companion computer (the folder name is from the Raspberry Pi version). They sit on top of two things that run separately:

- the **ZED SDK ROS 2 wrapper** (`zed_node`) — camera, visual odometry, spatial mapping. It needs the ZED SDK and CUDA (a Jetson); in the bisg_isaac sim and on its drone it is bisg_isaac's ZED stack, at `/drone_1/zed/zed_node`.
- **MAVROS** — either started from here (`mavros_launch.sh`) or, with bisg_isaac, its own `px4-bridge` service at `/drone_1/mavros`.

Topic inputs are relative names, so every node follows the drone namespace (`DRONE_NS`, e.g. `/drone_1`, applied as `--ros-args -r __ns:=...`). What the GCS reads (`/gcs/*`, `/brain/*`, `/barcode/*`) stays absolute.

---

## Deployment

Docker (preferred) — see the top-level README:

```bash
DRONE_NS=/drone_1 MAV_DEVICE=/dev/ttyTHS1 docker compose -f docker/compose.yaml up -d
```

On the host:

```bash
# MAVROS (+ extras for VFR_HUD / GPS status), GeographicLib datasets, rosbridge
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-rosbridge-server
sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh
sudo usermod -aG dialout $USER   # serial access to the flight controller

# Python dependencies
pip3 install --break-system-packages opencv-python pyzbar

# Install systemd services
sudo cp rosbridge.service  /etc/systemd/system/
sudo cp drone-gcs.service  /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now rosbridge.service

# Launch everything manually (optional — systemd handles it on boot)
bash start_all.sh --ns /drone_1
```

---

## Files

### `gcs_control.py` — Service Manager
ROS2 node that subscribes to `/gcs/cmd` and manages the lifecycle of every other node.  
The GCS app sends `{"action":"start","service":"barcode"}` — this node starts or stops the corresponding process (`mavros`, `brain`, `barcode`) and streams its status back on `/gcs/status` at 1 Hz.

**Environment:** `DRONE_NS` — namespace brain and barcode run in; `GCS_SERVICES` — comma list of the services the dashboard may start (the bisg profile sets `brain,barcode`: the sim owns MAVROS).

**Topics:**
- Sub: `/gcs/cmd` — `std_msgs/String` JSON commands
- Pub: `/gcs/status` — `std_msgs/String` JSON status map

---

### `zed_map_relay.py` — ZED Map for the GCS
The ZED wrapper's `mapping/fused_cloud` carries the whole map in every message — in the bisg warehouse 4.2 M points, 68 MB, once a second, far too much for rosbridge. This node voxel-downsamples it to xyz float32 and republishes it as `gcs/map_cloud` at most every 2 s (~120 k points, ~0.6 MB/s through rosbridge).

It subscribes to the fused cloud **only while `gcs/map_cloud` has a subscriber**, so the ZED stack ships nothing when no GCS has the map open. Started by `rosbridge_boot.sh` / `start_all.sh`.

| Parameter | Default | |
|---|---|---|
| `voxel` | 0.2 | m, downsampling grid |
| `max_points` | 200000 | hard cap (random subset above it) |
| `period` | 2.0 | s between published maps |

---

### `kinetix_bridge.py` — Kinetix Buttons
Backend of the GCS **Kinetix** tab. Runs in the `kinetix:jazzy` image with the Kinetix repo mounted at `/kinetix` (compose service `kinetix-bridge`, profile `kinetix`) and drives Kinetix only through its CLI, one job at a time:

| Command (`/gcs/kinetix/cmd`) | Runs |
|---|---|
| `{"action":"run","scene":"warehouse_room","method":"tour","seed":0}` | `python3 -m kinetix run --scene config/scenes/<scene>.yaml --method <m> --rig mavros --seed <n>` |
| `{"action":"sfm","run":"<run id>"}` (no run = newest) | `python3 -m kinetix sfm runs/<run>` |
| `{"action":"abort"}` | SIGINT to the job = Kinetix's Ctrl+C: fly home at a safe height, AUTO.LAND, disarm |

`run` is refused while anything else publishes `<DRONE_NS>/mavros/setpoint_position/local` — Kinetix streams the setpoints itself and only one producer may run per drone. Status (`/gcs/kinetix/status`, 1 Hz JSON): state (`idle`, `flying`, `aborting`, `reconstructing`), job, log tail, scenes, methods, recent runs with `run.yaml` summary and `sfm/offline/metrics.json`, other setpoint producers, last message.

---

### `barcode_detector.py` — Vision + Inventory
Subscribes to the ZED left image (JPEG), runs **pyzbar** barcode/QR detection on every frame (throttled to 4 fps to save CPU), and for each new detection:

1. Draws a green polygon + label overlay on the frame
2. Stamps the detection with the current **SLAM position** (x, y, z, θ) from the ZED odometry
3. Publishes a JSON detection event on `/barcode/detection`
4. Publishes the annotated image on `/barcode/roi/image/compressed`

**Dedup:** the same code is suppressed for 3 seconds to avoid duplicate log entries.

**SLAM overlay:** x / y / z / heading rendered on every frame in the top-left corner so operators can visually confirm position data.

**Topics:**
- Sub: `zed/zed_node/left/color/rect/image/compressed` (best-effort, depth 1)
- Sub: `zed/zed_node/odom` (ZED visual odometry)
- Pub: `/barcode/roi/image/compressed`
- Pub: `/barcode/detection` — `{"barcode":"P003GUA","slam_x":1.2,"slam_y":-0.5,"slam_z":1.1,"slam_theta":-116.8}`

---

### `brain_node.py` — Autonomous Mission Planner
Reads a waypoint mission from a **SQLite database** (`~/brain_data.db`), tracks the drone's position from the flight controller's EKF (`mavros/local_position/pose`, which fuses the ZED odometry), and streams `geometry_msgs/PoseStamped` setpoints in that same frame to `mavros/setpoint_position/local` at 10 Hz.

When the drone comes within `1.5 m` of a waypoint it advances to the next one. Mission progress is published to `/brain/planned_path` at 1 Hz for the GCS map to display.

On first run, seeds the DB with a default 5 × 5 m square mission. Edit with any SQLite client to define custom missions.

**Control loop:** 10 Hz setpoint → flight controller  
**Map update:** 1 Hz path publish → GCS

---

### `mavros_launch.sh` / `mavros_streams.py` — MAVROS
Runs **MAVROS** (`apm.launch`, ArduPilot plugin set) on the Pixhawk serial link. The port is auto-detected (`/dev/serial0 → ttyAMA10 → ttyAMA0 → ttyUSB0 → ttyACM0`; `ttyAMA10` comes before `ttyAMA0` because on the RPi5 `ttyAMA0` may be the Bluetooth UART).

`mavros_streams.py` runs alongside it and calls `/mavros/set_stream_rate` each time the FCU (re)connects — ArduCopter does not stream telemetry to a companion computer unless asked or unless the port's `SRx_` parameters are set.

**Topics used by the GCS:**

| Topic | Type | QoS | Source MAVLink message |
|---|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | reliable, transient-local | `HEARTBEAT` |
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | reliable | `VFR_HUD` (mavros_extras) |
| `/mavros/rc/out` | `mavros_msgs/RCOut` | reliable | `SERVO_OUTPUT_RAW` |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | reliable | `GPS_RAW_INT` (mavros_extras) |
| `/mavros/battery` | `sensor_msgs/BatteryState` | best-effort | `SYS_STATUS` / `BATTERY_STATUS` |
| `/mavros/imu/data` | `sensor_msgs/Imu` (ENU/FLU) | best-effort | `ATTITUDE` |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | best-effort | `GLOBAL_POSITION_INT` |
| `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` (FLU) | best-effort | `LOCAL_POSITION_NED` |

ROS2 subscribers to the best-effort topics must use `qos_profile_sensor_data` (or any best-effort profile) — a reliable subscriber receives nothing.

**Commands** (MAVROS services):
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode   mavros_msgs/srv/SetMode     "{custom_mode: GUIDED}"
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 5.0}"
ros2 service call /mavros/cmd/land    mavros_msgs/srv/CommandTOL "{}"
```

**Serial configuration:**
```bash
# Required for RPi5 GPIO UART (add to /boot/firmware/config.txt)
enable_uart=1

# Environment overrides
MAV_PORT=/dev/ttyAMA10   # serial port
MAV_BAUD=57600           # baud rate
```

---

### `start_all.sh` — Full Stack Launcher
One-command startup for all components. Accepts CLI flags to enable/disable individual modules:

```bash
bash start_all.sh [--ns /drone_1] [--mavros PORT] [--brain] [--barcode]
```

Starts: rosbridge → ZED map relay → (optionally) MAVROS, brain, barcode.  
All logs go to `~/gcs_logs/<timestamp>/`.

---

### `rosbridge.service` / `drone-gcs.service` — systemd Services
Enable rosbridge WebSocket server and gcs_control to start automatically on boot:

```bash
sudo systemctl enable rosbridge.service
sudo systemctl status rosbridge.service
```
