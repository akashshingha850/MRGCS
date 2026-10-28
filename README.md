# MRGCS — MavRos Ground Control Station

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
MRGCS/
├── main.js              ← Electron main process (window, menu, IPC)
├── preload.js           ← Renderer ↔ main bridge (contextBridge)
├── renderer/            ← Frontend application (runs in the Electron window)
│   ├── index.html       ← UI shell (tabs, panels, styles)
│   ├── app.js           ← All application logic (ROS subscriptions, charts, map)
│   └── lib/roslib.js    ← ROSLIB.js (WebSocket ↔ ROS bridge client)
├── docker/              ← Onboard stack image + compose (onboard, bisg sim)
├── onboard/             ← Onboard software (the companion computer: Raspberry Pi or Jetson)
│   ├── gcs_control.py       ← ROS2 service manager (start/stop nodes on demand)
│   ├── barcode_detector.py  ← Barcode/QR detection + ZED position overlay
│   ├── brain_node.py        ← Autonomous waypoint mission planner
│   ├── mavros_launch.sh     ← MAVROS launcher (serial auto-detect, ArduCopter)
│   ├── mavros_streams.py    ← Requests telemetry stream rates on FCU connect
│   ├── zed_map_relay.py     ← ZED spatial map → downsampled cloud for the GCS
│   ├── kinetix_bridge.py    ← Kinetix tab backend (runs the Kinetix CLI in its own image)
│   ├── sensor_check.py      ← Sensors tab backend (live sources, analyse flight, recording)
│   ├── sensor_analysis.py   ← Sensors tab analysis: which source EKF2 should fuse (numpy only)
│   ├── start_all.sh         ← One-command full stack launcher
│   ├── rosbridge_boot.sh    ← rosbridge_server startup script
│   ├── rosbridge.service    ← systemd service (auto-start on boot)
│   └── mrgcs.service        ← systemd service for gcs_control
└── docs/
    ├── Drone_GCS_Manual.pdf         ← Full technical manual
    ├── Drone_GCS_Presentation.pptx  ← Client presentation deck
    └── assets/                      ← Screenshots used in docs
```

---

## Application Modules

### 1 · Mission Control Dashboard
The entry point for every mission. Shows the live ZED camera feed, NAV map with the drone's real-time SLAM position and planned waypoints, motor thrust visualisation, battery status, and a service panel to start/stop any component. ARM/DISARM, takeoff, land and flight mode commands go to the flight controller through MAVROS services.

### 2 · Navigation
Full flight telemetry display: animated artificial horizon (AHRS), compass rose, and live Chart.js graphs for altitude, airspeed, and vertical speed. All data arrives from the standard MAVROS topics (`/mavros/state`, `/mavros/vfr_hud`, `/mavros/imu/data`, …); commands go through MAVROS services (`/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/cmd/takeoff`, `/mavros/cmd/land`).

### 3 · SLAM (ZED)
The tab is named SLAM, but it is **visual-inertial SLAM (VSLAM / VIO) from the ZED stereo camera and its IMU. There is no lidar**: pose comes from the ZED's visual-inertial tracking and the map from its stereo-depth spatial mapping.

Real-time 3D map rendered with WebGL, in the camera's own colour (the ZED's `rgb`, carried through the relay); a button switches to colouring by height (blue = low, red = high, scaled to the map's own height range). Save Map writes a `.pcd` with the colour. The view frames the whole map when it first arrives. The ZED wrapper's spatial mapping (`mapping/fused_cloud`, the whole map in one message — millions of points) is voxel-downsampled on board by `zed_map_relay.py` to ~120 k points every 2 s; the pose comes from the ZED's visual odometry (`zed_node/odom`). Mapping can be switched on/off, the pose reset (only while disarmed — the flight controller fuses the ZED odometry), and the map on screen saved to a `.pcd` file.

### 4 · Image Processing / Inventory Detection
The tab opens on the ZED's **RGB, depth and confidence** images (depth and confidence are colour-mapped JPEGs made on board by `zed_image_relay.py`; the raw 32-bit images are too big for Wi-Fi) next to the barcode camera with its detection overlay.

The most operationally valuable module. The `barcode_detector.py` node subscribes to the ZED left image, runs pyzbar detection on every frame, stamps each detection with the current ZED visual-odometry position (x, y, z), inserts a record into SQLite, and publishes the annotated frame back to the GCS. The result is a live inventory list with geolocated positions, exportable to CSV/Excel.


### 5 · Kinetix (whole-room photogrammetry)
Buttons for the Kinetix capture pipeline: pick a scene, method (`tour` ≈ 5 min, `roi_coverage` ≈ 25 min) and seed, for `tour`, Kinetix explores the room and scans it in one flight (a short exploration with the ZED depth fits the room's walls and obstacles, then the tour is planned on that instead of the scene file); pick the **Scan** arm for benchmarking: *optimized* (the exploration views count as coverage and go to SfM, the tour flies only the gaps) or *traditional* (exploration only maps, the tour scans the whole room); the run folder is named `..._tour-explore-optimized_...` / `..._tour-explore-traditional_...` and the runs table shows the arm; the tab shows the exploration progress, the fitted room (grey), the flight volume (dashed: the camera height band, so it stops well below a high ceiling), the obstacles it found (red) and the scene file's room (cyan dashed) to see how far off the fit is; **Start scan** (double-click — it arms and flies), **Abort** (Kinetix flies home, lands and disarms). The tab shows the job's state, elapsed time and live log; a **live flight** panel with the ZED left image (what Kinetix captures) and how well the drone holds the setpoint Kinetix streams (distance, speed = motion blur, height and yaw error against Kinetix's own capture tolerances, with an *in tolerance* indicator and a 30 s plot); a top-down **plan & result** view of the planned viewpoints, the path captured, the room box and, while flying, the drone, its trail and the setpoint; and a table of recent runs (size on disk, frames, stop reason, coverage, distance flown, wall time, and once reconstructed: images registered, camera error against ground truth, completeness); tick runs (or *all*) and **Delete selected** (click twice) to remove their folders — images, depth, rosbag, reconstruction; the run of a job that is running cannot be deleted. **Reconstruction**: pick a run, **Reconstruct** runs `kinetix sfm` on it (CPU, ~1-2 min for a tour; refused while a scan flies) and **Show in 3D** loads its sparse cloud into the **Scan 3D** tab with every camera coloured by its error against ground truth (green ≤ 0.25 m … red ≥ 1 m; white = the ground-truth centres) and a list of the worst cameras: one with few 3D points is weakly held and bundle adjustment can move it far off. Start is refused while anything else streams setpoints to the drone. Backed by `kinetix_bridge.py`, which runs the Kinetix CLI inside Kinetix's own Docker image.

---

### 6 · Sensors (which source EKF2 should trust)
Calibrations stay in QGC. This tab measures, on this drone in this room, which source EKF2 should fuse: **heading** (magnetometer vs the ZED: VIO, plus the bridge's wall compass when it is on), **height** (baro vs LW20 range vs ZED), **horizontal velocity** (optical flow vs ZED), and the **IMU** (noise at rest and hovering, PX4's vibration metrics and accel clipping, FCU vs ZED IMU). It also measures what Kinetix needs: the **ZED latency** (stamp delay behind the FCU gyro, i.e. the EV delay EKF2 should use), the **EV lever arm** (sim), how long the drone takes to settle inside **Kinetix's capture tolerances** after each step, the **motion blur** at those moments, the **EKF2 pose error at capture** (sim) and whether the **room** suits the ZED and the wall compass. Live: every source's rate and latest value, plus the EKF2 params now on the FCU.

**Analyse flight** (double-click) takes off in OFFBOARD and runs: 5 s on the ground (motors off), climb 1.5 m, hover, 16 yaw steps of 45°, a ±0.6 m height step, a 1 m square at 0.3 m/s (forward, left, back), land on the start point, 5 s on the ground. Every step holds 4 s, like a Kinetix capture (~3 min; needs ~2 m clear ahead and to the left; abort lands from where it is; a pilot mode change ends it). **Record by hand** records while you carry, turn and lift the drone with the motors off. The result is the **EKF2 params to set**, each with its reason, against the current ones, as `param set` lines for the QGC MAVLink console, followed by the tables behind it.

Without ground truth, headings are judged by their error per 45° step against the FCU gyro integrated over the step (well under 1° of gyro drift), which is where VIO slips and magnetometer deviation show. Heights are judged by a three-cornered hat over baro, range and ZED. Velocities use the same hat over flow, ZED and EKF2; this one is approximate, because EKF2 fuses the other two. Absolute checks: the height after landing on the take-off floor should be 0, and each source's start → landing displacement can be compared with a tape measurement of the real landing offset. In the sim, Pegasus' ground truth decides instead. Its magnetometer is an ideal Earth field, so only the real drone gives a magnetometer verdict. Each run is saved to `/data/sensor_runs/<time>-<kind>.npz` (+ `.json`) and can be re-analysed offline with `python3 onboard/sensor_analysis.py <run>.npz`. Backed by `sensor_check.py` (the node: live, flight, recording) and `sensor_analysis.py` (numpy only, `tests/test_sensor_analysis.py`). Vibration needs MAVROS' `vibration` plugin: bisg_isaac `MAVROS_PLUGINS=lean+debug`.

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

Without Docker: install `ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-rosbridge-server`, run `sudo /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh` and `pip3 install --break-system-packages opencv-python pyzbar`, then `bash onboard/start_all.sh --ns /drone_1` (or enable `rosbridge.service`).

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
