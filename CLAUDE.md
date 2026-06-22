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

There are two node implementations in `cc02_autodrive/`, representing successive iterations of the same controller — both subscribe to GNSS position and publish Ackermann drive commands:

- **`pid_controller.py`** (`PidController`) — earliest skeleton. Subscribes to `/gnss/solution` (`gnss_ros_standardization/msg/GnssSolution`), logs the received ENU position, and publishes a constant stop command (`speed=0`, `steering_angle=0`) to `/ackermann_cmd` on a 0.1s timer. No actual control logic — used to validate the GNSS→node wiring.

- **`pid_controller_v2.py`** (`PidControllerV2`) — the real controller. Key behaviors:
  - Loads a list of `(x, y)` ENU waypoints from a CSV file (`waypoint_file` param, columns `x,y`; see `waypoints_example.csv`).
  - On each `GnssSolution` message (topic `/gnss/solution`):
    - Ignores `status == 0` (invalid fix).
    - Estimates heading from consecutive ENU positions (`atan2(dy, dx)`), only updating when movement exceeds 5cm to filter GNSS noise.
    - Checks distance to current waypoint against `waypoint_radius`; advances `waypoint_index` and resets the PID integral/prev-error state on arrival.
    - Runs a manual PID loop on heading error (target bearing to waypoint minus current heading, normalized to [-π, π]) to compute `steering_angle`, clamped to `max_steering_angle`.
    - Selects speed based on GNSS fix quality: `speed_fix` when `status == 1` (RTK FIX), else `speed_float` (covers FLOAT/SPP/etc).
  - A 0.5s timer (`_safety_check`) publishes a stop command if no GNSS message has been received for >0.5s.
  - All tunables (waypoint file/radius, speeds, PID gains, max steering angle) are declared as ROS 2 parameters, overridable via `--ros-args -p key:=value`.

**GNSS status convention** (from `GnssSolution.status`, defined in `gnss_ros_standardization`): `0`=invalid, `1`=FIX, `2`=FLOAT, `5`=SPP (single point positioning).

## Known gotchas

- `setup.py` only registers `pid_node` → `pid_controller:main` (the v1 stub) as a console script entry point. `pid_controller_v2:main` is **not** registered, so `ros2 run cc02_autodrive <name>` won't launch v2 until an entry point is added.
- `waypoint_file` defaults to the relative path `'waypoints.csv'`, which resolves against the process's current working directory, not the package share directory. `waypoints_example.csv` is also not listed in `setup.py`'s `data_files`, so it isn't installed/discoverable via `ament_index`/share dir lookup — pass an absolute path via the ROS param when running.
- Depends on the `gnss_ros_standardization` package's custom message `GnssSolution`; that package must be built first in the same workspace.
