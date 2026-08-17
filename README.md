# jerk_reducer

A ROS package that limits the **jerk** (rate of change of acceleration) of velocity commands sent to a mobile robot, producing smoother, less jarring motion when paired with local planners such as **DWA** or **TEB**.

Sudden changes in commanded acceleration — common right after obstacle avoidance maneuvers or abrupt planner replans — translate into jerky, uncomfortable robot motion and higher mechanical stress. `jerk_reducer` sits between the local planner and `cmd_vel`, clipping tangential and lateral jerk to configurable limits in real time, while still respecting velocity and acceleration bounds.

## Features

- Real-time jerk limiting on both **tangential** and **lateral** components of motion
- Configurable via `dynamic_reconfigure` — tune limits live without restarting nodes
- Drop-in node that subscribes to a raw `cmd_vel` and republishes a smoothed, bounded command
- Built-in **data loggers** for offline analysis and controller comparison:
  - `jerk_data_logger.py` — logs raw vs. filtered commands, jerk, odometry, and cross-track error when running with `jerk_reducer`
  - `baseline_jerk_logger.py` — logs the same schema directly from the planner output, with no jerk reducer in the loop, for apples-to-apples comparison
- Cross-track error logging against the global path (nearest-point distance)
- Tested against both **DWA** and **TEB** local planners, with and without obstacles

## How It Works

At each control step, the reducer bounds the tangential and lateral jerk of the commanded velocity, then integrates forward to produce a smoothed acceleration and velocity that respect both jerk and acceleration/velocity limits:

```
v̈_k  = clip(v̈_k,  -j_tan_max + v_(k-1)·ω_(k-1)²,  j_tan_max + v_(k-1)·ω_(k-1)²)
v̇_k  = clip(v̇_(k-1) + v̈_k·dt,  -v̇_max,  v̇_max)
v_k   = clip(v_(k-1) + v̇_k·dt,  0,  v_max)

ω̇_k  = clip(ω̇_k,  (-j_lat_max - 2·v̇_k·ω_(k-1)) / v_(k-1),  (j_lat_max - 2·v̇_k·ω_(k-1)) / v_(k-1))
ω_k   = clip(ω_(k-1) + ω̇_k·dt,  -ω_max,  ω_max)
```

`clip(x, a, b)` is the standard saturation operator, `max(a, min(b, x))`, applied to keep each derivative within its physical limit at every timestep.

## Package Layout

```
jerk_reducer/
├── cfg/
│   └── JerkReducer.cfg          # dynamic_reconfigure parameters
├── include/jerk_reducer/
├── src/
│   ├── jerk_reducer.cpp         # core jerk-limiting library
│   ├── main.cpp                 # node entry point
│   ├── jerk_data_logger.py      # logger for jerk_reducer runs
│   └── baseline_jerk_logger.py  # logger for baseline (no reducer) runs
├── launch/
├── config/
└── CMakeLists.txt
```

## Dependencies

- `roscpp`
- `geometry_msgs`
- `nav_msgs`
- `dynamic_reconfigure`
- Python 2 (for the logger nodes, ROS Melodic/Noetic compatible)

## Build

```bash
cd ~/catkin_ws/src
git clone <repo-url> jerk_reducer
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

Launch the node with your planner's raw `cmd_vel` remapped as input:

```bash
roslaunch jerk_reducer jerk_reducer.launch
```

Tune limits live with `rqt_reconfigure`:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

### Sending a navigation goal (for testing)

```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "{
  header: {stamp: now, frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### Logging data for comparison

```bash
# With jerk_reducer active
rosrun jerk_reducer jerk_data_logger.py

# Baseline, planner output only
rosrun jerk_reducer baseline_jerk_logger.py
```

Both loggers write timestamped CSVs to `~/jerk_logs` by default (configurable via the `~output_dir` param), including commanded/filtered velocity, tangential and lateral jerk, odometry, and cross-track error relative to the global path.

## Benchmark Results

Tested with DWA and TEB local planners, with and without an obstacle in the path. The jerk-bounded controller consistently reduces RMS and maximum jerk, and tangential/lateral acceleration, with a modest trade-off in navigation time in some scenarios.

**DWA — RMS Tangential Jerk (m/s³):** Controller 14.82 → **Jerk Bound 9.57** (no obstacle), Controller 13.40 → **Jerk Bound 9.20** (with obstacle)

**TEB — RMS Tangential Jerk (m/s³):** Controller 20.70 → **Jerk Bound 13.25** (no obstacle), Controller 24.18 → **Jerk Bound 12.42** (with obstacle)

Maximum tangential jerk is capped at the configured limit (20.0 m/s³) in all jerk-bounded runs, versus uncapped peaks of 60–120 m/s³ in the unfiltered controller output.

## License

Apache License, Version 2.0
