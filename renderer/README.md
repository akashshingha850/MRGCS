# renderer/ — GCS Frontend (Electron)

The Drone GCS user interface — a single-page Electron renderer that connects to the Raspberry Pi over a rosbridge WebSocket and visualises all telemetry, video, and sensor data in real time.

No ROS installation is required on the operator's laptop. The app runs on **Windows, Linux, and macOS**.

---

## Files

### `index.html` — UI Shell
Defines the four-tab layout, all panel structure, and global styles. Tabs:

| Tab | Content |
|---|---|
| **DASHBOARD** | Live ZED camera, NAV map, ARM/DISARM, service controls, battery, motors |
| **NAVIGATION** | Artificial horizon, compass, altitude/speed/vspeed charts, flight data table |
| **SLAM** | ZED spatial map (WebGL point cloud), ZED VIO pose, mapping on/off, PCD export |
| **IMAGE PROCESSING** | Barcode camera stream, detection inventory list, DB log, volumetry panel |
| **KINETIX** | Whole-room photogrammetry: start a scan (scene / method / seed), abort → land, reconstruct, live log, recent runs with SfM metrics (`/gcs/kinetix/*`, `kinetix_bridge.py`) |

---

### `app.js` — Application Logic

All runtime behaviour: ROS connections, subscriptions, UI updates, and user interactions.

#### Connection
Connects to `ws://<host>:9090` via **ROSLIB.js**. The host is read from the top-bar input field. Reconnects automatically on disconnect.

#### Namespaces and autopilot
Every 5 s the app lists the topics through rosapi and takes the namespaces from `*/mavros/state`, `*/zed_node/odom` and `*/gcs/map_cloud` (`/mavros`, `/zed/zed_node` on a plain setup; `/drone_1/mavros`, `/drone_1/zed/zed_node` in bisg_isaac), re-subscribing when one changes. The autopilot comes from `vehicle_info_get`: for PX4 the mode buttons send PX4 modes (GUIDED → `OFFBOARD`, LOITER → `AUTO.LOITER`, AUTO → `AUTO.MISSION`, STAB → `STABILIZED`, RTL → `AUTO.RTL`), LAND uses `AUTO.LAND`, and TAKEOFF is refused (PX4 takes an AMSL altitude). When `vfr_hud` is not published, speed / climb / altitude come from `local_position` and heading from the IMU.

#### Dashboard
- **ARM / DISARM / mode buttons** → call `/mavros/cmd/arming` / `/mavros/set_mode`; the command log shows the service result (OK / FAIL)
- **TAKEOFF / LAND / RTL** → call `/mavros/cmd/takeoff` / `/mavros/cmd/land` / `/mavros/set_mode`
- **Live camera (`ZED — Left`)** → `<ZED_NS>/left/color/rect/image/compressed` (JPEG) drawn on a canvas, throttled by rosbridge to 10 fps (~2 MB/s at HD1080)
- **NAV map** → Canvas drawn at 20 Hz from `/brain/planned_path`; shows real path (blue), planned waypoints (dashed yellow), drone arrow
- **Service panel** → subscribes to `/gcs/status`, publishes to `/gcs/cmd` on button click.
- **Battery ring** → updates from `/mavros/battery`
- **Motor circles** → 5-motor layout, thrust derived from `/mavros/rc/out` PWM values

#### Navigation
- **Artificial horizon** → Canvas redrawn on each `/mavros/imu/data` message; pitch shifts horizon line, roll rotates the disc (MAVROS ENU/FLU attitude is converted to aviation NED angles)
- **Compass rose** → SVG-style Canvas, heading from `/mavros/vfr_hud`
- **Charts** → Chart.js rolling 60-sample window for altitude, airspeed, vertical speed

#### SLAM
- **Point cloud** → `<ns>/gcs/map_cloud`, the ZED spatial map downsampled on board by `zed_map_relay.py` (each message is the whole map and replaces the view). Points coloured by height using a red→yellow→green→blue gradient
- **Pose** → `<ZED_NS>/odom` (ZED visual odometry), also drives the Dashboard NAV map
- **Controls** → Map On / Off: `<ZED_NS>/enable_mapping` (`std_srvs/SetBool`); Reset Pose: `<ZED_NS>/reset_pos_tracking` (`std_srvs/Trigger`), refused while armed — the flight controller fuses the ZED odometry, so a reset in flight is a position jump; Save Map: downloads the map on screen as an ASCII `.pcd`; Clear Map: clears the view until the next map arrives

#### Image Processing
- **Barcode camera** → ROS topic `/barcode/roi/image/compressed` decoded from base64 JPEG
- **Detection list** → builds a live table from `/barcode/detection` events (time, code, product, position)
- **DB log** → shows the last 5 INSERT confirmations
- **Export** → `Exportar CSV / Excel` button serialises the in-memory detection list

#### Barcode source
The barcode detector reads the ZED image directly, so starting the **CODIS** (barcode) service needs nothing else running on the GCS side.

---

### `lib/roslib.js` — ROSLIB.js
Vendored copy of [roslibjs](https://github.com/RobotWebTools/roslibjs). Provides `ROSLIB.Ros`, `ROSLIB.Topic`, and `ROSLIB.Message` — the WebSocket client that bridges JavaScript to the ROS2 topic graph.

---

## ROS Topics Consumed

MAVROS names are shown under `/mavros`; the prefix is whatever namespace was detected.

| Topic | Type | Used in |
|---|---|---|
| `/mavros/vfr_hud` | `mavros_msgs/VfrHud` | Navigation — heading, speed, altitude |
| `/mavros/local_position/pose`, `velocity_local` | `PoseStamped`, `TwistStamped` | Navigation — fallback when there is no `vfr_hud` |
| `/mavros/state` | `mavros_msgs/State` | Status bar — armed/mode |
| `/mavros/rc/out` | `mavros_msgs/RCOut` | Dashboard — motor thrust rings |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | Navigation — GPS fix/sats |
| `/mavros/battery` | `sensor_msgs/BatteryState` | Dashboard — battery ring |
| `/mavros/imu/data` | `sensor_msgs/Imu` | Navigation — horizon, attitude |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Dashboard — lat/lon/alt abs |
| `/mavros/local_position/velocity_body` | `geometry_msgs/TwistStamped` | Navigation — Vx/Vy/Vz (body FLU) |
| `/brain/planned_path` | `std_msgs/String` (JSON) | Dashboard — NAV map |
| `<ZED_NS>/left/color/rect/image/compressed` | `sensor_msgs/CompressedImage` | Dashboard — camera |
| `<ZED_NS>/odom` | `nav_msgs/Odometry` | SLAM tab + NAV map — pose |
| `<ns>/gcs/map_cloud` | `sensor_msgs/PointCloud2` | SLAM tab — map |
| `/barcode/detection` | `std_msgs/String` (JSON) | Image Processing — inventory |
| `/barcode/roi/image/compressed` | `sensor_msgs/CompressedImage` | Image Processing — camera |
| `/gcs/status` | `std_msgs/String` (JSON) | Dashboard — service indicators |

## Running as a web page

The renderer needs nothing from Electron (`preload.js` bridges nothing), so it also runs in any browser. The onboard container serves it on `:8090`; served over http(s), the default rosbridge URL is `ws://<page host>:9090`.

## ROS Topics Published

| Topic | Type | Trigger |
|---|---|---|
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Send Setpoint (GUIDED) |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | VEL STOP (zero velocity) |

## ROS Services Called

(ZED: `<ZED_NS>/enable_mapping`, `<ZED_NS>/reset_pos_tracking` — see SLAM above.)

| Service | Type | Trigger |
|---|---|---|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | ARM, DISARM |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Mode buttons, RTL, HOLD |
| `/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | TAKEOFF |
| `/mavros/cmd/land` | `mavros_msgs/CommandTOL` | LAND |
| `/gcs/cmd` | `std_msgs/String` (JSON) | Start/stop service buttons |
