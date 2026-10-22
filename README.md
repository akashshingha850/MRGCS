# Drone GCS — Autonomous Ground Control Station

> A full-stack UAV ground control system built on **Electron + ROS2**, running on the drone's **companion computer** with a **ZED stereo camera** and streaming all telemetry, video, and map data to a laptop over Wi-Fi in real time.

<p align="center">
  <img src="docs/assets/dashboard.png" alt="Dashboard" width="49%"/>
  <img src="docs/assets/slam.png" alt="SLAM" width="49%"/>
</p>
<p align="center">
  <img src="docs/assets/navigation.png" alt="Navigation" width="49%"/>
  <img src="docs/assets/barcode.png" alt="Barcode Detection" width="49%"/>
</p>

---

## What it does

The drone carries a companion computer running a ROS2 stack, connected to:

- A **ZED stereo camera** (ZED SDK ROS 2 wrapper) → video, visual-inertial odometry, and a live 3D spatial map; the same images feed barcode detection
- A **flight controller** (ArduCopter or PX4) over UART → MAVROS reads telemetry and sends commands

All data is bridged to the ground laptop over Wi-Fi via **rosbridge WebSocket**. The laptop runs a native **Electron desktop app** that visualises everything in real time — no ROS installation needed on the laptop.

The system can autonomously fly a programmed waypoint route, scan a warehouse for barcodes, and produce a geolocated inventory report — all without a human operating the sticks.

---

## Tech Stack

| Layer | Technology |
|---|---|
| Ground app | Electron + Node.js, ROSLIB.js, Chart.js, Leaflet |
| Communication | rosbridge WebSocket (port 9090) |
| Onboard middleware | ROS2 Jazzy, Docker (`docker/`) |
| Camera, VIO, 3D mapping | ZED SDK ROS 2 wrapper (`zed_node`: left image, odometry, spatial mapping) |
| Flight controller | ArduCopter or PX4 via MAVROS (MAVLink v2) |
| Vision | OpenCV + pyzbar (barcode/QR detection) |
| Storage | SQLite3 (inventory database) |
| Build target | Windows / Linux / macOS (Electron Builder) |

---

## Repository Structure

```
drone-gcs/
├── main.js              ← Electron main process (window, menu, IPC)
├── preload.js           ← Renderer ↔ main bridge (contextBridge)
├── renderer/            ← Frontend application (runs in the Electron window)
│   ├── index.html       ← UI shell (tabs, panels, styles)
│   ├── app.js           ← All application logic (ROS subscriptions, charts, map)
│   └── lib/roslib.js    ← ROSLIB.js (WebSocket ↔ ROS bridge client)
├── docker/              ← Onboard stack image + compose (onboard, bisg sim)
├── raspberry/           ← Onboard software (companion computer; folder name kept from the Pi era)
│   ├── gcs_control.py       ← ROS2 service manager (start/stop nodes on demand)
│   ├── barcode_detector.py  ← Barcode/QR detection + SLAM overlay
│   ├── brain_node.py        ← Autonomous waypoint mission planner
│   ├── mavros_launch.sh     ← MAVROS launcher (serial auto-detect, ArduCopter)
│   ├── mavros_streams.py    ← Requests telemetry stream rates on FCU connect
│   ├── zed_map_relay.py     ← ZED spatial map → downsampled cloud for the GCS
│   ├── kinetix_bridge.py    ← Kinetix tab backend (runs the Kinetix CLI in its own image)
│   ├── start_all.sh         ← One-command full stack launcher
│   ├── rosbridge_boot.sh    ← rosbridge_server startup script
│   ├── rosbridge.service    ← systemd service (auto-start on boot)
│   └── drone-gcs.service    ← systemd service for gcs_control
└── docs/
    ├── Drone_GCS_Manual.pdf         ← Full technical manual
    ├── Drone_GCS_Presentation.pptx  ← Client presentation deck
    └── assets/                      ← Screenshots used in docs
```

---

## Application Modules

### 1 · Mission Control Dashboard
The entry point for every mission. Shows the live ZED camera feed, NAV map with the drone's real-time SLAM position and planned waypoints, motor thrust visualisation, battery status, and a service panel to start/stop any component. ARM/DISARM and flight mode commands send directly to the flight controller via MAVLink.

### 2 · Navigation
Full flight telemetry display: animated artificial horizon (AHRS), compass rose, and live Chart.js graphs for altitude, airspeed, and vertical speed. All data arrives from the standard MAVROS topics (`/mavros/state`, `/mavros/vfr_hud`, `/mavros/imu/data`, …); commands go through MAVROS services (`/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/cmd/takeoff`, `/mavros/cmd/land`).

### 3 · SLAM (ZED)
Real-time 3D map rendered with WebGL, coloured by height (red = high, blue = low). The ZED wrapper's spatial mapping (`mapping/fused_cloud`, the whole map in one message — millions of points) is voxel-downsampled on board by `zed_map_relay.py` to ~120 k points every 2 s; the pose comes from the ZED's visual odometry (`zed_node/odom`). Mapping can be switched on/off, the pose reset (only while disarmed — the flight controller fuses the ZED odometry), and the map on screen saved to a `.pcd` file.

### 4 · Image Processing / Inventory Detection
The most operationally valuable module. The `barcode_detector.py` node subscribes to the ZED left image, runs pyzbar detection on every frame, stamps each detection with the current SLAM position (x, y, z), inserts a record into SQLite, and publishes the annotated frame back to the GCS. The result is a live inventory list with geolocated positions, exportable to CSV/Excel.


### 5 · Kinetix (whole-room photogrammetry)
Buttons for the Kinetix capture pipeline: pick a scene, method (`tour` ≈ 5 min, `roi_coverage` ≈ 25 min) and seed, **Start scan** (double-click — it arms and flies), **Abort** (Kinetix flies home, lands and disarms), and **Reconstruct** any run (`kinetix sfm`). The tab shows the job's state, elapsed time and live log, and a table of recent runs with frames, coverage, registered frames, 3D points, reprojection error and ATE. Start is refused while anything else streams setpoints to the drone. Backed by `kinetix_bridge.py`, which runs the Kinetix CLI inside Kinetix's own Docker image.

---

## System Architecture

![System Architecture](docs/architecture.png)

---

## Quick Start

### Run the onboard stack (Docker)

On the drone's companion computer, next to a running ZED wrapper (`zed_node` publishing the left image, odometry and spatial mapping):

```bash
docker compose -f docker/compose.yaml build
DRONE_NS=/drone_1 MAV_DEVICE=/dev/ttyTHS1 docker compose -f docker/compose.yaml up -d
```

The `onboard` container runs rosbridge (:9090), the ZED map relay and `gcs_control`; MAVROS, brain and barcode start from the dashboard inside it. `DRONE_NS` is the namespace of the ZED / MAVROS topics (empty when they are at `/zed/...` and `/mavros/...`). The ZED wrapper itself is not in the image — it needs the ZED SDK and CUDA.

Without Docker: install `ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-rosbridge-server`, run `sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh` and `pip3 install --break-system-packages opencv-python pyzbar`, then `bash raspberry/start_all.sh --ns /drone_1` (or enable `rosbridge.service`).

### Test against the bisg_isaac simulator

With a bisg_isaac sim running (PX4 SITL, its MAVROS at `/drone_1/mavros` and its ZED stack at `/drone_1/zed/zed_node`):

```bash
docker/bisg_up.sh        # onboard stack + Kinetix bridge on the sim's ROS domain, DRONE_NS=/drone_1; `docker/bisg_up.sh down` to stop
```

Then open `http://localhost:8090` (or connect the desktop app to `ws://localhost:9090`). The Kinetix bridge starts when the `kinetix:jazzy` image and the Kinetix repo (`KINETIX_DIR`, default `../kinetix` next to this repo) exist. The app finds the MAVROS and ZED namespaces and the autopilot (PX4 / ArduPilot) on its own and maps the mode buttons to PX4 modes. The sim owns MAVROS, so the Services panel offers brain and barcode only. bisg_isaac's lean MAVROS plugin list has no `vfr_hud`, `rc_io` or `gps_status`, so speed / climb / altitude come from the EKF local position, and the motor rings and GPS badge stay empty.

### Open the GCS

**In a browser — nothing to install.** The onboard container serves the app: open `http://<host>:8090` (`http://localhost:8090` on the machine running `docker/bisg_up.sh`; any device on the LAN or tailnet works the same way). The page connects to rosbridge on the host it was loaded from.

**Or as a desktop app** (Electron, same code):

```bash
npm install
npm start              # development
npm run build          # build installer (Win/Linux/macOS)
```

Set the drone's IP in the app connection bar: `ws://<DRONE_IP>:9090`

---

## Key Design Decisions

**Why MAVROS?** It is the standard MAVLink ↔ ROS2 bridge, so the GCS speaks the same topics and services as any other MAVROS-based vehicle, commands return real success/failure, and the brain node's `/mavros/setpoint_position/local` setpoints reach the flight controller. `mavros_launch.sh` picks the serial port before starting MAVROS (on a Raspberry Pi 5, `ttyAMA0` may be the Bluetooth UART).

**Why Electron?** The client app needs to run natively on Windows/Linux/macOS without requiring any ROS installation on the operator's machine. Electron + ROSLIB.js gives a full desktop UI that connects to any ROS2 robot over a standard WebSocket.

**Why the ZED?** One sensor gives the video, the visual-inertial odometry the flight controller fuses indoors (no GPS), and the spatial map. The ZED SDK needs an NVIDIA GPU (CUDA), so the companion computer is a Jetson, not a Raspberry Pi.

---

## Hardware

| Component | Model |
|---|---|
| Onboard computer | NVIDIA Jetson (the ZED SDK needs CUDA) |
| Camera | Stereolabs ZED (e.g. ZED Mini) |
| Flight controller | Pixhawk / Pixracer (ArduCopter or PX4) |
| Frame | Custom quadcopter |
| GCS | Any Windows / Linux laptop |
