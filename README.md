# autonav_pkg

`autonav_pkg` is a ROS 1 navigation demonstration for a differential-drive
robot in a simulated static environment. It combines simulated LiDAR,
odometry, occupancy-grid mapping, and an Artificial Potential Field (APF)
controller.

## What it demonstrates

- A URDF/Xacro robot model with a LiDAR sensor
- Synthetic obstacles published as visualization markers
- Conversion of obstacle markers into a `/scan` stream
- `/cmd_vel` integration into `/odom` and TF
- Occupancy-grid construction from scan and odometry data
- Goal-oriented APF navigation with a `/set_goal` service

APF is a local-planning method. Local minima and incomplete global navigation
are expected limitations of this demonstration.

## Prerequisites

- Ubuntu with ROS 1 and a configured catkin workspace
- `roscpp`, `rospy`, `std_msgs`, `urdf`, `xacro`
- `message_generation` and `message_runtime`
- RViz for visualization

The package metadata targets ROS 1; select a ROS distribution compatible with
your installed catkin environment.

## Build and run

Place this package under the `src/` directory of a catkin workspace:

```bash
cd ~/catkin_ws/src
git clone <repository-url> autonav_pkg
cd ..
catkin_make
source devel/setup.bash
roslaunch autonav_pkg autonav.launch
```

Set a goal from another sourced terminal:

```bash
rosservice call /set_goal "x: 2.0
y: 3.0"
```

Open the configured RViz view from the launch file to inspect the robot,
obstacles, LiDAR scan, map, TF, and path.

## Package layout

```text
autonav_pkg/
├── config/       # Runtime parameters
├── launch/       # Simulation launch files
├── rviz/         # RViz configurations
├── src/          # Navigation and simulation nodes
├── srv/          # SetGoal.srv service definition
├── xacro/        # Robot description
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Nodes

| Node | Responsibility |
| --- | --- |
| `complex_obstacles.py` | Publishes static obstacle markers |
| `lidar.py` | Publishes simulated `LaserScan` data |
| `odom_node.py` | Integrates velocity commands and publishes odometry/TF |
| `map.py` | Builds an occupancy grid |
| `apf_controller.py` | Computes APF commands and serves navigation goals |
| `path.py` | Publishes path visualization |

## Troubleshooting

- Empty map: verify `/scan`, `/odom`, and the required TF frames are active.
- No movement: confirm the controller is publishing `/cmd_vel` and that the
  goal service call succeeded.
- Stuck robot: APF local minima are a known limitation; tune parameters in
  `config/` or replace the local planner with a global-planning strategy.

## Development

Run `git diff --check` before submitting documentation or source changes.
Keep generated catkin build directories outside commits.

## License

See [LICENSE](./LICENSE).
