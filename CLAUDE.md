# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`cc02_autodrive` is a ROS 2 (ament_python) package implementing GNSS-waypoint-following autonomous driving via PID steering control, for a research project ("研究活動:PID制御による自律走行"). It is one of several packages built together inside a `ros2_ws` colcon workspace, alongside `gnss_ros_standardization` (provides GNSS position/status messages) and `rc_car_driver` (low-level vehicle actuation).

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
  - **Starts driving automatically as soon as the GNSS ENU origin is resolved** (first valid FIX/FLOAT fix) — no manual start trigger.
  - Checks distance to the current waypoint against `wp_radius`; advances `waypoint_index` and resets the PID integral/derivative state on arrival.
  - Runs a manual PID loop on heading error (target bearing to waypoint minus current heading, normalized to [-π, π]) to compute `steering_angle`, clamped to `max_steering_angle`.
  - Selects speed based on GNSS fix quality: `speed_fix` when FIX, else `speed_float` (FLOAT).
- A 0.1s timer (`_safety_check`) publishes a stop command if no GNSS message has been received for `gnss_timeout_s` seconds.
- All tunables (waypoint file, radius, speeds, PID gains, max steering angle, timeouts) are declared as ROS 2 parameters, overridable via `--ros-args -p key:=value`.

**GNSS status convention** (from `GnssSolution.status`, defined in `gnss_ros_standardization`): `0`=invalid, `1`=FIX, `2`=FLOAT, `5`=SPP (single point positioning).

`cc02_autodrive/stanley_controller.py` (`StanleyController`) is an alternative controller node (entry point `stanley_node`), independent of `pid_node` — same waypoint loading / heading-bootstrap / safety-stop structure, but follows a spline-smoothed path with the Stanley steering law instead of the PID heading-error loop:

- After the ENU origin is resolved, the sparse CSV waypoints (converted to ENU) are fit with a parametric cubic spline (`scipy.interpolate.splprep`, `s=0`) and resampled by arc length into a dense path at `path_spacing` (default 0.05 m = 5 cm) intervals. The dense path's tangent (`splev(..., der=1)`) gives the desired heading at each point. This turns the professor's ~18 corner points into a smooth curve the vehicle tracks continuously, rather than cutting corners between straight segments.
- Each callback finds the nearest dense-path point ahead of the vehicle (forward windowed search, `search_window` points, to prevent snapping backward), then applies `steering_angle = heading_error + atan2(stanley_k * cross_track_error, k_soft + current_speed)`, where `heading_error` is the path tangent heading minus vehicle heading and `cross_track_error` is the signed perpendicular distance from the vehicle to that point.
- No integral/derivative terms — it's a feed-forward geometric law evaluated fresh each callback, with `k_soft` softening the cross-track term at low speed.
- Original (sparse) waypoint passages are still announced (`★ WP[i] 通過！`), triggered when the nearest path index crosses the dense-path index closest to each original waypoint (robust to GNSS noise; `wp_radius` is no longer used for advancement). Stops when the nearest index reaches the path end within `goal_tolerance`.
- Tunable via `path_spacing`, `stanley_k` (cross-track gain), `k_soft`, `goal_tolerance`, `search_window`, plus the same `speed_fix`/`speed_float`/`max_steering_angle`/`bootstrap_speed`/`min_speed_for_heading`/`max_speed_mps`/`gnss_timeout_s` params as `pid_node`. Requires `numpy` and `scipy`.

Only one of `pid_node` / `stanley_node` should be run at a time — both publish to `/ackermann_cmd`.

`cc02_autodrive/gnss_logger.py` (`GnssLogger`, entry point `gnss_logger_node`) subscribes to `/gnss/solution` and appends every received message to a CSV (`log_file` param; defaults to a timestamped filename in the cwd, flushed after every row so `Ctrl-C` never loses data). Columns: wall/ROS/GPS timestamps, status, num_sats, ratio, hdop, latitude/longitude/altitude, ENU x/y, speed. Included in `autodrive_bringup.launch.py` by default, writing to `~/ros2_ws/gnss_logs/gnss_log_<launch time>.csv` (override via the `log_file` launch argument).

`cc02_autodrive/plot_log_map.py` (entry point `plot_log_map`, plain argparse script — not an rclpy node) renders a `gnss_logger` CSV onto an interactive Folium/OpenStreetMap HTML map: a gray polyline for the full track, FIX/FLOAT/other points as separate toggleable colored layers (green/orange/red), start/end markers, and an optional `--wp-file` overlay of the original sparse waypoints as purple flag markers. Usage: `ros2 run cc02_autodrive plot_log_map <log.csv> [--wp-file <wp_position.csv>] [-o <out.html>]`. Requires `folium` (installed via `apt install python3-folium`, not pip — this venv is externally-managed/PEP 668).

## Known gotchas

- `wp_file` defaults to the relative path `'wp_position.csv'`, which resolves against the process's current working directory, not the package share directory — pass an absolute path (or the `wp_file` launch argument) when running outside the launch file.
- Depends on the `gnss_ros_standardization` package's custom message `GnssSolution`; that package must be built first in the same workspace.
