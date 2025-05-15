# Assignment: PA4 - Environment Mapper

#### Author: Megan Liu

#### Class: COSC81.01 Principles of Robot Design and Programming

#### Instructor: Alberto Quattrini Li

#### Term: Spring 2025

#### Description: This code implements environment mapping behavior for ROSbot2. It takes in data from a lidar sensor and creates/updates an occupancy grid of its environment.

## Requirements

- ROS2 -- tested on ROS2 humble, but other versions may work.

## How to run:

1. [Terminal 1] run docker using `docker compose up`
2. [Terminal 2] enter pa4 directory
3. [Terminal 3] visualize map in rviz > add topic > choose map

```

rviz2

```

7. [Terminal 8] run PA4

```

python3 pa4-mapping.py

```

4. Terminal 4

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
