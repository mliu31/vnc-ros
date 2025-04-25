1. run docker using docker compose up
2. close rosbot-gazebo in docker desktop
3. install stage (lightweight simulator) in a new terminal
   docker compose exec ros bash
   cd pa2
   bash install_stage.sh
   source ../install/setup.bash
4. start the empty world or corridor world
   ros2 launch stage_ros2 stage.launch.py world:=/root/catkin_ws/src/pa2/empty enforce_prefixes:=false one_tf_tree:=true
   ros2 launch stage_ros2 stage.launch.py world:=/root/catkin_ws/src/pa2/2017-02-11-00-31-57 enforce_prefixes:=false one_tf_tree:=true
