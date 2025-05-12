#!/usr/bin/env python

# Author: Megan Liu
# Date: 2025/05/11
# PA4: Environment Mapper
# Class: CS81 Robotics with AQL 

import tf_transformations
import tf2_ros # library for transformations.
from tf2_ros import TransformException


import rclpy # module for ROS APIs
from rclpy.node import Node
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist 
from rclpy.duration import Duration # message type for duration

import numpy as np
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

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.height = height
        self.width  = width

    def cell_at(self, r, c):
        return self.grid[r, c]
    
    def is_valid(self, r,c): 
        r,c = int(r), int(c)
        buffer = 8

        if 0 <= r < self.height and 0 <= c < self.width and not self.grid[r-buffer:r+buffer,c-buffer:c+buffer].any():
            return True 
        
        return False

class Plan(Node):
    def __init__(self, map_frame_id=MAP_FRAME_ID, node_name=NODE_NAME, context=None):
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL, 
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # map subscriber and info 
        self.map = None # the variable containing the map.
        self.map_frame_id = map_frame_id
        
        # occupancy grid 
        self.sub = self.create_publisher(OccupancyGrid, MAP_TOPIC, 1)
        self.occgrid_frame_id = None

        # Setting up transformation listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)



    def get_transformation(self, start_frame, target_frame):
        """Get transformation between two frames."""
        try:
            while not self.tf_buffer.can_transform(target_frame, start_frame, self.get_clock().now()):
                print("waiting for transform...")
                rclpy.spin_once(self)
            tf_msg = self.tf_buffer.lookup_transform(target_frame, start_frame, self.get_clock().now())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')
            return
    
        # self.get_logger().info(f'Received tf message: {tf_msg}')   
        translation = tf_msg.transform.translation
        quaternion = tf_msg.transform.rotation

        t = tf_transformations.translation_matrix([translation.x, translation.y, translation.z])
        R = tf_transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        T = t.dot(R)
        T = np.round(T, decimals=1) # rounding minimizes noise 
        
        return T, quaternion

    # pos, orientation relative to map rf
    def get_currentloc(self):
        map_T2_bl, quaternion = self.get_transformation(TF_BASE_LINK, TF_MAP)
        print("map_T2_bl:\n", map_T2_bl)

        # position 
        bl_p = np.array([0, 0, 0, 1])
        map_p = map_T2_bl.dot(bl_p.transpose())
        pose = self.create_pose(map_p[0], map_p[1], map_p[2], quaternion)

        return pose
    
    def get_angle(self, y, x):
        theta = math.atan2(y, x)

        # edge cases for tan (multiples of π/2)
        if x == 0:
            if y > 0:
                theta = math.pi/2
            else:
                theta = -math.pi/2

        return theta 
    
    def make_quat(self, yaw):
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def px_to_pose(self, curr_px, prev_pose): 
        x, y = self.px_to_grid(curr_px[0], curr_px[1])
        px, py, _ = self.pose_to_grid(prev_pose)
        # print("         processing ", px, py, " --> ", x, y)

        # angle to rotate from prev coords --> curr coords 
        angle = self.get_angle(y-py, x-px)
        
        # print("          angle: ", angle)

        q = self.make_quat(angle)
        pose = self.create_pose(x, y, 0, q) 

        return pose

    def pose_to_grid(self, pose): 
        return (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)

    def path_to_pxposes(self, path, start_pose): # in px
        print("     converting px path to poses")
        pose_seq = [start_pose] 

        # track angle throughout poses
        quaternion = start_pose.pose.orientation 
        rpy = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        yaw = rpy[2]
        
        for i in range(1, len(path)):
            p = self.px_to_pose(path[i], pose_seq[i-1])
            pose_seq.append(p)
        
        return pose_seq
            
    def publish_posearray(self, poses_arr): 
        print("   publishing poses array")
        pa = PoseArray()

        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = self.map_frame_id
        pa.poses = [ps.pose for ps in poses_arr]

        self.pose_seq_pub.publish(pa)
        print("   published")
    
    def px_to_grid(self,x_w, y_w): # px to m
        # column index
        c = x_w * self.map.resolution
        # row index
        r = y_w * self.map.resolution
        
        return c, r  

    def grid_to_px(self,x, y): # m to pixels
        # column index
        r = x // self.map.resolution
        # row index
        c = y // self.map.resolution
        return r,c

    def cleanup_path(self, path): 
        # include start, end, rotations 
        clean_path = []

        # segment_start point 
        segment_start = path[0]
        matching_coord = None
        for i in range(len(path)): 
            coord = path[i]

            # if start or end, add to clean_path 
            if i == 0 or i == len(path)-1: 
                clean_path.append(path[i])
                continue 

            if matching_coord == None:
                # if x of coord matches segment_start point, 
                if path[i][0] == segment_start[0]: 
                    matching_coord = "x"
                else: 
                    matching_coord = "y"
            else:  
                x_coord_match = matching_coord == "x" and coord[0] == segment_start[0]  
                y_coord_match =  matching_coord == "y" and coord[1] == segment_start[1]

                if x_coord_match or y_coord_match: 
                    continue 
                else: 
                    clean_path.append(path[i])
                    segment_start = path[i]
                    matching_coord = None 

        return clean_path

    # on pixels 
    def treesearch(self, start, end, stack):  # with history 
        print("   TREESEARCH from ", start, " --> ", end)       
    
        visited = set()  
        prev = {start:None}  
        frontier = [start]

        while len(frontier) > 0: 
            if stack: 
                leaf = frontier.pop()
            if not stack: 
                leaf = frontier.pop(0)
            # print("leaf: ", leaf)
            visited.add(leaf)

            if leaf == end: 
                # backtrack through history to get path 
                node = end
                path = []
                while node != None: 
                    path.append(node)
                    node = prev[node]
                path.reverse()
                # print("   PX PATH::: ", path)

                return path 
            
            # add neighbors (decided 4 arbitrarily, not 8)
            x, y = leaf 
            neighbors = [(x+1,y), (x-1,y), (x,y+1), (x,y-1)]  # dfs order - down, up, left, right
            for n in neighbors: 
                c,r = n
                if self.map.is_valid(r,c) and n not in visited:  # valid bounds
                    # print("   neighbor: ", n)
                    prev[n] = leaf
                    frontier.append(n)
                    visited.add(n)

        return None 
    

def main(args=None):
    # 1st. initialization of node.
    rclpy.init(args=args)

    p = Plan()

    # wait until the map callback has run
  

        rclpy.spin_once(p)
        rclpy.spin_once(e)
        break 
        
    rclpy.shutdown()

if __name__ == "__main__":
    main()
