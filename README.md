# Autonav_pkg — Autonomous Navigation Demo (ROS)

This package demonstrates the autonomous navigation of a differential-drive robot in a simulated static environment using ROS (Robot Operating System). The robot is equipped with a LiDAR sensor for real-time obstacle detection and mapping. The system integrates several core components, including a URDF robot model, LiDAR-based perception, odometry, occupancy grid mapping, and a goal-setting service. Navigation and obstacle avoidance are achieved using an Artificial Potential Field (APF) controller, which generates attractive forces toward the target and repulsive forces away from obstacles to ensure smooth and safe motion. This package provides a complete end-to-end pipeline for robotic navigation, making it ideal for education, demonstration, and as a base for future robotics research.

**Students:** Beilassan Hdewa, Lana Al wazzeh, Zain Alabidin Shbani

---

## Overview
This ROS package demonstrates a complete simulated pipeline for autonomous navigation of a differential-drive robot. The system includes:

- URDF robot model (differential drive chassis + LiDAR)
- Simulated environment publisher (`complex_obstacles_node`)
- LiDAR emulator that converts published obstacles to `/scan` using TF
- Odometry node that integrates `/cmd_vel` into `/odom` and publishes TF
- Occupancy grid map builder from `/scan` and `/odom`
- APF (Artificial Potential Field) controller that subscribes to `/scan` & `/odom` and publishes `/cmd_vel`
- A service (`/set_goal`) to set navigation goals

This repo is intended for education, demonstration, and as a base for further research.

---

## Features / Highlights
- Real-time obstacle avoidance using APF (local planner).
- Occupancy grid mapping and visualization in RViz.
- TF frame publishing for `world` → `body`.
- Simple stuck-recovery routine.
- Modular ROS nodes so components can be swapped/extended.

---

## Repository structure

```text
autonav_pkg/
│
├─ package.xml
├─ CMakeLists.txt
├─ README.md
├─ LICENSE
├─ .gitignore
│
├─ launch/
│ └─ autonav.launch
│
├─ rviz/
│ └─ autonav.rviz
│
├─ xacro/
│ └─ differential_robot.xacro
│
├─ srv/
│ └─ SetGoal.srv
│
├─ scripts/
│ ├─ complex_obstacles.py
│ ├─ lidar.py
│ ├─ odom_node.py
│ ├─ map.py
│ ├─ path.py
│ ├─ arduino.py
│ └─ apf_controller.py
│
├─ config/
│ └─ params.yaml
│
└─ docs/
└─ screenshots/
```
---

## Quick start (run demo)
1. Make sure you have a catkin workspace and the package is placed in `src/`.
2. Build:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
3. Make Python scripts executable:
```bash
chmod +x src/autonav_pkg/src/*.py
```
4. Launch everything:
```
roslaunch autonav_pkg autonav.launch
```
5. Open RViz (if not auto-started) and set `Fixed Frame = world`. Visualize `robot_description`, `/scan`, `/map`, `/odom`, `/obstacles`, `TF`, and `Path`.
6. Set a goal using the service (example with rosservice):
```
rosservice call /set_goal "x: 5.0 y: 5.0"
```
## Nodes

- complex_obstacles_node — publishes static obstacles as `visualization_msgs/MarkerArray` on `/obstacles`.
- lidar_from_markers_tf — listens to `/obstacles` + `world->body TF` and publishes `/scan` (LaserScan).
- odom_node — subscribes to `/cmd_vel`, integrates motion, publishes `/odom` and `TF`.
- map_builder — subscribes to `/scan` & `/odom` and publishes `/map` (OccupancyGrid).
- apf_controller — subscribes to `/scan` & `/odom`, exposes `/set_goal` service, and publishes `/cmd_vel`.

##  Messages / Services

- Service srv/goal.srv:
```
float64 x
float64 y
---
int32 success
string message
```


## Troubleshooting & Notes

- If RViz shows empty map: confirm `/scan` and `/odom` are publishing and TF world->body exists.
- If robot stuck in local minima: APF is a local planner — consider hybrid global planner or randomized escape behavior.
- The odometry here is simulated by integrating `/cmd_vel` (no encoder noise/correction).

## Suggested further work

- Replace local APF with a hybrid planner (global path + local obstacle avoidance).
- Add SLAM (gmapping / Cartographer) for robust localization.
- Plug in real robot hardware by replacing the lidar_from_markers_tf and odom_node with real topics.

## Authors

Beilassan Hdewa, Lana Al wazzeh, Zain Alabidin Shbani

## License

This project is licensed under the MIT License — see LICENSE for details.


