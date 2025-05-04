# Assignment: PA3 - Path Planner

#### Author: Megan Liu

#### Class: COSC81.01 Principles of Robot Design and Programming

#### Instructor: Alberto Quattrini Li

#### Term: Spring 2025

#### Description: This code implements shape drawing behavior for the ROSbot2 using map_server. It plans a path and executes it.

## Requirements

- ROS2 -- tested on ROS2 humble, but other versions may work.

## How to run:

1. [Terminal 1] run docker using `docker compose up`
2. close rosbot-gazebo in docker desktop
3. [Terminal 2] enter pa3 directory (with the map files) and run the lightweight simulator Stage (installation instructions in PA2 if needed)

```docker compose exec ros bash
 cd pa3
 ros2 launch stage_ros2 stage.launch.py world:=/root/catkin_ws/src/pa3/maze
enforce_prefixes:=false one_tf_tree:=true
```

4. [Terminal 3] in a new terminal, run start the map server

```
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=pa3/maze.yml
```

5. [Terminal 3.5] visualize map

```
ros2 run rviz2 rviz2
```

6. [Terminal 5,6] start the loading of the map and publish the tf map-odom
   (install map_server first)

```
sudo apt update &&
sudo apt install ros-humble-nav2-map-server
```

```
ros2 run nav2_util lifecycle_bringup map_server
ros2 run tf2_ros static_transform_publisher 2 2 0 0 0 0 map odom --ros-args -p use_sim_time:=true
```

7. [Terminal 8] run PA3

```
python3 pa3-planning.py
```

8. [Terminal 9] send a fresh map message

```
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{ map_url: 'pa3/maze.yml' }"
```
