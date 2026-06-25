# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`cc02_autodrive` is a ROS 2 (ament_python) package implementing GNSS-waypoint-following autonomous driving via PID steering control, for a research project ("研究活動:PID制御による自律走行"). It is one of several packages built together inside a `ros2_ws` colcon workspace, alongside `gnss_ros_standardization` (provides GNSS position/status messages) and `rc_car_driver` (low-level vehicle actuation).

**RTK architecture (as of 2026-06-24): computed inside the u-blox receiver, not on the Pi.** RTCM corrections are pushed into the receiver over a second serial port (UART1, `/dev/ttyAMA0`) via RTKLIB's `str2str`. `ubx_driver_node` reads the receiver's own internally-computed fix via NAV-PVT over USB and publishes it straight to `/gnss/solution` (`ubx_driver.yaml`: `messages.nav_pvt: true`, `solution_topic: "/gnss/solution"`). `rtcm_decoder_node`/`real_time_kinematic` (Pi-side RTKLIB computation, an earlier iteration) are no longer used. See [[project-ubx-internal-rtk-plan]] in memory for the full background/rationale.

**Launch files are split in two (as of 2026-06-24)** so the GNSS side can be left running continuously while the control side is restarted freely (e.g. switching `controller`, re-testing a route) without waiting for RTK to re-converge each time:
- `launch/gnss_bringup.launch.py`: `ubx_driver_node` + `str2str` (the latter via `ExecuteProcess`, so `Ctrl-C` stops both together). NTRIP credentials are read from the `NTRIP_STREAM_PATH` environment variable by default (`ntrip_stream_path` launch arg overrides it) — kept out of git-tracked files on purpose.
- `launch/control_bringup.launch.py`: the controller (`controller` arg: `pid`/`stanley`/`pure_pursuit`), `vehicle_driver`, `gnss_logger_node`. Run `gnss_bringup.launch.py` first (once, leave it running) and re-launch `control_bringup.launch.py` as needed.

## Build / Test Commands

Run from the workspace root (e.g. `~/ros2_ws`), not from inside this package directory:

```bash
colcon build --packages-select cc02_autodrive
source install/setup.bash
```

`--symlink-install` は使わないこと: `cc02_autodrive`の `console_scripts`エントリポイント（`pid_node`）が`.egg-link`経由のインストールになり、対応する`easy-install.pth`が生成されないため、`importlib.metadata`がパッケージメタデータを解決できず`ros2 launch`実行時に`PackageNotFoundError`で確実に落ちる（`ros2 run`単体では再現しないため見つけにくい）。

Run tests (flake8, pep257, copyright linters via ament):

```bash
colcon test --packages-select cc02_autodrive
colcon test-result --verbose
```

To run a single lint check directly with pytest:

```bash
python3 -m pytest test/test_flake8.py -v
python3 -m pytest test/test_pep257.py -v
```

Run a node:

```bash
ros2 run cc02_autodrive pid_node
```

## Architecture

`cc02_autodrive/pid_controller.py` (`PidController`) is the controller node (entry point `pid_node`). Key behaviors:

- Loads a list of `(lat, lon, height)` waypoints from a CSV file (`wp_file` param; columns `WP,Latitude(deg),Longitude(deg),Ellipsoidal Height(m)`), converted to local ENU `(x, y)` once the GNSS ENU origin is resolved from the first valid fix.
- On each `GnssSolution` message (topic `/gnss/solution`):
  - Ignores statuses outside FIX/FLOAT (invalid/SPP/SBAS/DGPS etc.).
  - Estimates heading from the velocity vector (`vel_enu`, course-over-ground via `atan2`), only trusting it above `min_speed_for_heading` to filter GNSS/Doppler noise.
  - **Starts driving only after the first RTK FIX is observed since node startup** (`fix_achieved` flag, set once and never cleared) — the ENU origin / waypoint path may resolve earlier on a FLOAT fix, but `_control()` holds neutral until a FIX has occurred at least once. After that initial FIX, FLOAT fixes during the run are tolerated as usual (no re-stop). No manual start trigger.
  - Checks distance to the current waypoint against `wp_radius`; advances `waypoint_index` and resets the PID integral/derivative state on arrival. On reaching the last waypoint, **loops**: wraps `waypoint_index` back to `1` instead of stopping, so the route (typically authored to end back at its own start point) is driven indefinitely until the process is killed or `_safety_check` trips.
  - Runs a manual PID loop on heading error (target bearing to waypoint minus current heading, normalized to [-π, π]) to compute `steering_angle`, clamped to `max_steering_angle`.
  - Selects speed based on GNSS fix quality: `speed_fix` when FIX, else `speed_float` (FLOAT).
- A 0.1s timer (`_safety_check`) publishes a stop command if no GNSS message has been received for `gnss_timeout_s` seconds.
- All tunables (waypoint file, radius, speeds, PID gains, max steering angle, timeouts) are declared as ROS 2 parameters, overridable via `--ros-args -p key:=value`.

**GNSS status convention** (from `GnssSolution.status`, defined in `gnss_ros_standardization`): `0`=invalid, `1`=FIX, `2`=FLOAT, `5`=SPP (single point positioning).

`cc02_autodrive/stanley_controller.py` (`StanleyController`) is an alternative controller node (entry point `stanley_node`), independent of `pid_node` — same waypoint loading / heading-bootstrap / safety-stop / straight-line-segment structure as `pid_node` (no spline), but with the Stanley steering law instead of the PID heading-error loop:

- Waypoints are connected by straight-line segments. `segment_start` tracks the start of the line currently being followed (the vehicle's position at startup, or the just-reached waypoint after each advance); the target is `waypoints[waypoint_index]`. `path_heading = atan2(ty - sy, tx - sx)` is this segment's bearing.
- `steering_angle = heading_error + atan2(stanley_k * cross_track_error, k_soft + current_speed)`, where `heading_error` is `path_heading` minus vehicle heading and `cross_track_error` is the signed perpendicular distance from the vehicle to the segment.
- No integral/derivative terms — it's a feed-forward geometric law evaluated fresh each callback, with `k_soft` softening the cross-track term at low speed.
- Same `wp_radius`-based arrival/advance/announce and end-of-route infinite-loop behavior (wrap `waypoint_index` back to `1`, carrying `segment_start` forward to the just-reached point) as `pid_node`.
- Tunable via `stanley_k` (cross-track gain), `k_soft`, plus the same `wp_radius`/`speed_fix`/`speed_float`/`max_steering_angle`/`bootstrap_speed`/`min_speed_for_heading`/`max_speed_mps`/`gnss_timeout_s` params as `pid_node`.

`cc02_autodrive/pure_pursuit_controller.py` (`PurePursuitController`) is a third alternative controller node (entry point `pure_pursuit_node`). Unlike `stanley_node`, it does **not** use a spline — waypoints are connected by straight-line segments (same `wp_radius`-based arrival/advance/loop structure as `pid_node`), with the Pure Pursuit geometric steering law instead of PID's heading-error loop:

- Lookahead distance is speed-adaptive: `Ld = lookahead_min + lookahead_gain * current_speed`. `_lookahead_target()` walks forward from `waypoint_index` along the straight segments (current position → `waypoints[waypoint_index]` → `waypoints[waypoint_index+1]` → ...), linearly interpolating within whichever segment first accumulates `Ld` of distance — correctly spanning multiple short segments when `Ld` exceeds the distance to the next waypoint, and clamping to the last waypoint past the route's end.
- `steering_angle = atan2(2 * wheelbase_m * sin(alpha), Ld_actual)`, where `alpha` is the angle between vehicle heading and the bearing to the lookahead point, and `Ld_actual` is the true Euclidean distance to that point (not the nominal `Ld`). This is the standard bicycle-model Pure Pursuit law.
- `wheelbase_m` (front-to-rear axle distance) defaults to **0.267 m, measured on the actual chassis.**
- Same waypoint-passage announcements and end-of-route infinite-loop behavior (wrap `waypoint_index` back to `1`) as `pid_node`.
- Tunable via `wheelbase_m`, `lookahead_min`, `lookahead_gain`, plus the same `wp_radius`/`speed_fix`/`speed_float`/`max_steering_angle`/`bootstrap_speed`/`min_speed_for_heading`/`max_speed_mps`/`gnss_timeout_s` params as `pid_node`.

Only one of `pid_node` / `stanley_node` / `pure_pursuit_node` should be run at a time — all three publish to `/ackermann_cmd`. `control_bringup.launch.py`'s `controller` launch argument (`pid`/`stanley`/`pure_pursuit`) selects between them via conditional `Node` actions.

`cc02_autodrive/gnss_logger.py` (`GnssLogger`, entry point `gnss_logger_node`) subscribes to `/gnss/solution` and appends every received message to a CSV (`log_file` param; defaults to `gnss_log_latest.csv` in the cwd, flushed after every row so `Ctrl-C` never loses data). Columns: wall/ROS/GPS timestamps, status, num_sats, latitude/longitude/altitude, ENU x/y, speed. `ratio`/`hdop` columns were removed (2026-06-24) since under the u-blox-internal RTK architecture (see above) they're always `0.0`/`NaN` — `ratio` has no NAV-PVT equivalent, and `hdop` requires `messages.nav_dop: true` in `ubx_driver.yaml`, which isn't enabled. Included in `control_bringup.launch.py` by default, writing to `~/ros2_ws/gnss_logs/gnss_log_latest.csv` — **a fixed filename that gets overwritten on every launch** (changed from a timestamped name on 2026-06-24, per explicit request to stop accumulating log files); pass a different `log_file` launch argument value if you want to keep a specific run's log before it's overwritten.

`matlab/plot_log_map.m` (plain MATLAB function, not part of the ROS package/build — replaced the earlier Folium/Python `plot_log_map.py` on 2026-06-24) renders a `gnss_logger` CSV on a MATLAB `geoaxes` satellite basemap: a gray polyline for the full track, FIX/FLOAT/other points as separate colored scatter series (green/orange/red), start/end markers. An optional second argument overlays the original sparse waypoints as magenta markers plus a blue straight-line polyline connecting them in file order — mirrors what `pid_node`/`stanley_node`/`pure_pursuit_node` actually drive (no spline). Usage (in MATLAB): `plot_log_map('gnss_log_xxxx.csv', 'wp_position_basic.csv')`. Requires MATLAB Mapping Toolbox. **MATLAB Desktop does not run on the Raspberry Pi (ARM Linux) this workspace lives on** — copy the CSV to a Windows/Mac/x86-64-Linux machine with MATLAB installed and run it there. No HTML export or distance-measuring tool (unlike the old Folium version) — use MATLAB's own pan/zoom and `distance()` for that.

## Known gotchas

- `wp_file` defaults to the relative path `'wp_position_basic.csv'`, which resolves against the process's current working directory, not the package share directory — pass an absolute path (or the `wp_file` launch argument) when running outside the launch file. Two route files ship in `data_files`: `wp_position_basic.csv` (WP0→...→WP8→WP0, default) and `wp_position_advance.csv` (longer loop via WP9-17); both loop-close (last row duplicates the first).
- Depends on the `gnss_ros_standardization` package's custom message `GnssSolution`; that package must be built first in the same workspace.
