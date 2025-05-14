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
2. close rosbot-gazebo in docker desktop
3. [Terminal 2] enter pa3 directory (with the map files) and run the lightweight simulator Stage (installation instructions in PA2 if needed)

```docker compose exec ros bash
 ros2 launch stage_ros2 stage.launch.py world:=/root/catkin_ws/src/pa3/maze enforce_prefixes:=false one_tf_tree:=true
```

```

5. [Terminal 3.5] visualize map in rviz > add topic > choose map

```

rviz2

```

7. [Terminal 8] run PA3

```

python3 pa3-planning.py

```

```
