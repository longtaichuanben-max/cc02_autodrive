# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

`cc02_autodrive` is a ROS 2 (ament_python) package implementing GNSS-waypoint-following autonomous driving via PID steering control, for a research project ("研究活動:PID制御による自律走行"). It is one of several packages built together inside a `ros2_ws` colcon workspace, alongside `gnss_ros_standardization` (provides GNSS position/status messages) and `rc_car_driver` (low-level vehicle actuation).

## Build / Test Commands

Run from the workspace root (e.g. `~/ros2_ws`), not from inside this package directory:

```bash
colcon build --packages-select cc02_autodrive --symlink-install
source install/setup.bash
```

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
  - **Starts driving automatically the first time RTK FIX is observed** — no manual start trigger. Until then it holds neutral.
  - Checks distance to the current waypoint against `wp_radius`; advances `waypoint_index` and resets the PID integral/derivative state on arrival.
  - Runs a manual PID loop on heading error (target bearing to waypoint minus current heading, normalized to [-π, π]) to compute `steering_angle`, clamped to `max_steering_angle`.
  - Selects speed based on GNSS fix quality: `speed_fix` when FIX, else `speed_float` (FLOAT).
- A 0.1s timer (`_safety_check`) publishes a stop command if no GNSS message has been received for `gnss_timeout_s` seconds.
- All tunables (waypoint file, radius, speeds, PID gains, max steering angle, timeouts) are declared as ROS 2 parameters, overridable via `--ros-args -p key:=value`.

**GNSS status convention** (from `GnssSolution.status`, defined in `gnss_ros_standardization`): `0`=invalid, `1`=FIX, `2`=FLOAT, `5`=SPP (single point positioning).

## Known gotchas

- `wp_file` defaults to the relative path `'wp_position.csv'`, which resolves against the process's current working directory, not the package share directory — pass an absolute path (or the `wp_file` launch argument) when running outside the launch file.
- Depends on the `gnss_ros_standardization` package's custom message `GnssSolution`; that package must be built first in the same workspace.
