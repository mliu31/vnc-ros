#!/usr/bin/env python

# Author: Megan Liu
# PA Template
# Spring 2025 CS81 Robotics 

import rclpy # module for ROS APIs
from rclpy.node import Node
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html

import math

NODE_NAME = "mapper"

MAP_TOPIC = "map"
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
OCCGRID_TOPIC = 'occupancy_grid'

TF_BASE_LINK = 'base_link'
TF_ODOM = 'odom'
TF_MAP = 'map'

MAP_FRAME_ID = "map"

USE_SIM_TIME = True

LINEAR_VELOCITY = 0.1 # m/s
ANGULAR_VELOCITY = math.pi/15 # rad/s

class PA(Node):
    def __init__(self, map_frame_id=MAP_FRAME_ID, node_name=NODE_NAME, context=None):
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL, 
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

    def spin(self):
        while rclpy.ok():
            print("Spinning")
            rclpy.spin_once(self)

    

def main(args=None):
    # 1st. initialization of node.
    rclpy.init(args=args)

    # Initialization of the class for the random walk.
    PA_node = PA()

    interrupted = False

    # Robot random walks.
    try:
        PA_node.spin()
    except KeyboardInterrupt:
        interrupted = True
        PA_node.get_logger().error("ROS node interrupted.")
    finally:
        # workaround to send a stop
        # thread.join()
        if rclpy.ok():
            PA_node.stop()

    if interrupted:
        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        PA_node = PA_node(node_name="random_walk_end", context=new_context)
        PA_node.get_logger().error("ROS node interrupted.")
        PA_node.stop()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()
