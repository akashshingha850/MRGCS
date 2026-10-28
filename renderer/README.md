# renderer/ — GCS Frontend (Electron)

The MRGCS (MavRos Ground Control Station) user interface — a single-page Electron renderer that connects to the drone's companion computer (Raspberry Pi or Jetson) over a rosbridge WebSocket and visualises all telemetry, video, and sensor data in real time.

No ROS installation is required on the operator's laptop. The app runs on **Windows, Linux, and macOS**.

---

## Files

### `index.html` — UI Shell
Defines the four-tab layout, all panel structure, and global styles. Tabs:

| Tab | Content |
|---|---|
| **DASHBOARD** | Live ZED camera, NAV map, ARM/DISARM, service controls, battery, quadcopter motor knobs (X frame, PX4 order M1 front-right, M2 rear-left, M3 front-left, M4 rear-right; thrust shown by the arc and needle, no numbers) |
| **NAVIGATION** | Artificial horizon, compass, altitude/speed/vspeed charts, flight data table |
| **SLAM** | Visual-inertial SLAM from the ZED stereo camera (no lidar): spatial map as a WebGL point cloud, ZED VIO pose, mapping on/off, PCD export |
| **IMAGE PROCESSING** | ZED RGB, depth and confidence images (depth / confidence as JPEGs from `zed_image_relay.py`), the barcode camera, detection inventory list, DB log, volumetry panel |
| **KINETIX** | Whole-room photogrammetry: start a scan (scene / method / seed / explore-first), abort → land, live log, live flight panel (ZED left image, FCU / battery / ZED health, distance / speed / height / yaw error vs Kinetix's capture tolerances, 30 s plot), plan & result map (planned viewpoints, captured path, room box, live drone / trail / setpoint), recent runs, reconstruction (`kinetix sfm` on a run) and the **Scan 3D** view of its sparse cloud with the cameras coloured by error against ground truth (`/gcs/kinetix/*`, `kinetix_bridge.py`) |

---

### `app.js` — Application Logic

All runtime behaviour: ROS connections, subscriptions, UI updates, and user interactions.

#### Connection
Connects to `ws://<host>:9090` via **ROSLIB.js**. The host is read from the top-bar input field. Reconnects automatically on disconnect.

#### Namespaces and autopilot
Every 5 s the app lists the topics through rosapi and takes the namespaces from `*/mavros/state`, `*/zed_node/odom` and `*/gcs/map_cloud` (`/mavros`, `/zed/zed_node` on a plain setup; `/drone_1/mavros`, `/drone_1/zed/zed_node` in bisg_isaac), re-subscribing when one changes. The autopilot comes from `vehicle_info_get`: for PX4 the mode buttons send PX4 modes (GUIDED → `OFFBOARD`, LOITER → `AUTO.LOITER`, AUTO → `AUTO.MISSION`, STAB → `STABILIZED`, RTL → `AUTO.RTL`), LAND uses `AUTO.LAND`, and TAKEOFF is refused (PX4 takes an AMSL altitude). When `vfr_hud` is not published, speed / climb / altitude come from `local_position` and heading from the IMU.

#### Dashboard
- **ARM / DISARM / mode buttons** → call `/mavros/cmd/arming` / `/mavros/set_mode`; the command log shows the service result (OK / FAIL)
- **TAKEOFF / LAND / RTL** → ArduCopter: `/mavros/cmd/takeoff` / `/mavros/cmd/land` / `/mavros/set_mode`. PX4: `MAV_CMD_NAV_TAKEOFF` takes an AMSL altitude, which a GPS-denied vehicle does not have, so TAKEOFF sets `MIS_TAKEOFF_ALT` from the Alt field (1-50 m above where it is, `/mavros/param/set`) and enters `AUTO.TAKEOFF` (the vehicle must be armed first; the Alt field is a height above the take-off point); LAND is `AUTO.LAND`, RTL `AUTO.RTL`
- **Live camera (`ZED — Left`)** → `<ZED_NS>/left/color/rect/image/compressed` (JPEG) drawn on a canvas, throttled by rosbridge to 10 fps (~2 MB/s at HD1080)
- **NAV map** → Canvas drawn at 20 Hz from `/brain/planned_path`; shows real path (blue), planned waypoints (dashed yellow), drone arrow
- **Service panel** → subscribes to `/gcs/status`, publishes to `/gcs/cmd` on button click.
- **Battery ring** → updates from `/mavros/battery`
- **Motor circles** → 5-motor layout, thrust derived from `/mavros/rc/out` PWM values

#### Navigation
- **Artificial horizon** → Canvas redrawn on each `/mavros/imu/data` message; pitch shifts horizon line, roll rotates the disc (MAVROS ENU/FLU attitude is converted to aviation NED angles)
- **Compass rose** → SVG-style Canvas, heading from `/mavros/vfr_hud`
- **Charts** → Chart.js rolling 60-sample window for altitude, airspeed, vertical speed

#### SLAM (visual-inertial, ZED camera — no lidar)
- **Point cloud** → `<ns>/gcs/map_cloud`, the ZED spatial map downsampled on board by `zed_map_relay.py` (each message is the whole map and replaces the view). Points are drawn in the camera's colour (`rgb`, bytes B,G,R,A); the *Colour* button switches to a height ramp scaled to the map's 2nd–98th percentile of z. The camera frames the map on the first message and again on Clear Map
- **Pose** → `<ZED_NS>/odom` (ZED visual odometry), also drives the Dashboard NAV map
- **Controls** → Map On / Off: `<ZED_NS>/enable_mapping` (`std_srvs/SetBool`); Reset Pose: `<ZED_NS>/reset_pos_tracking` (`std_srvs/Trigger`), refused while armed — the flight controller fuses the ZED odometry, so a reset in flight is a position jump; Save Map: downloads the map on screen as an ASCII `.pcd` (with `rgb` when the map has colour); Clear Map: clears the view until the next map arrives
- **Touch** → one finger rotates the 3D view, two fingers pinch-zoom and pan (mouse: drag to rotate, right-drag to pan, wheel to zoom)

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

The renderer needs nothing from Electron (`preload.js` bridges nothing), so it also runs in any browser. The onboard container serves it on `:8090`; served over http(s), the default rosbridge URL is `ws://<page host>:9090`. Any device on the same network or Tailscale tailnet can open `http://<host or Tailscale IP>:8090`.

### Opening a tab directly

`http://<host>:8090/?tab=kinetix` (also `dashboard`, `navigation`, `slam`, `image`) opens that tab on load, e.g. as a phone bookmark.

### Phone layout

Below 900 px width (`@media (max-width: 900px)` at the end of the `<style>` block in `index.html`) the shell becomes one scrolling column: the top bar keeps the logo and connection badges on one row and the tabs on a swipeable row below, every tab stacks its panels vertically, and each canvas gets an explicit height (canvases are sized from their parent, so they need one). Inputs are 16 px so iOS Safari does not zoom on focus, and `touch-action: manipulation` disables double-tap zoom so the double-tap on ARM and Start scan works. The desktop layout is unchanged above 900 px.

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
| `/mavros/cmd/takeoff` | `mavros_msgs/CommandTOL` | TAKEOFF (ArduCopter) |
| `/mavros/param/set` | `mavros_msgs/ParamSetV2` | TAKEOFF (PX4: `MIS_TAKEOFF_ALT`, then mode `AUTO.TAKEOFF`) |
| `/mavros/cmd/land` | `mavros_msgs/CommandTOL` | LAND |
| `/gcs/cmd` | `std_msgs/String` (JSON) | Start/stop service buttons |
