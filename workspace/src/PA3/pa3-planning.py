#!/usr/bin/env python

# Author: Megan Liu
# Date: 2025/03/31

import numpy as np

import tf_transformations
import tf2_ros # library for transformations.
from tf2_ros import TransformException

import rclpy # module for ROS APIs
from rclpy.node import Node
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

NODE_NAME = "planner"

MAP_TOPIC = "map"
POSE_TOPIC = "pose"
POSE_SEQUENCE = "pose_sequence"

TF_BASE_LINK = 'base_link'
TF_ODOM = 'odom'
TF_MAP = 'map'

MAP_FRAME_ID = "map"

USE_SIM_TIME = True

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution

    def cell_at(self, r, c):
        return self.grid[r, c]
    
    def inGrid(self, r,c): 
        try: 
            self.cell_at(r,c)
            return True 
        except: 
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

        self.sub = self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_callback, 1)
        self.pose_pub = self.create_publisher(PoseStamped, POSE_TOPIC, 1)
        self.pose_pub = self.create_publisher(PoseStamped, POSE_SEQUENCE, 1)
        self.map = None # the variable containing the map.
        self.map_frame_id = map_frame_id
        self.occgrid_frame_id = None

        # Setting up transformation listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def map_callback(self, msg):
        print("MAP CALLBACK")
        self.map = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        self.occgrid_frame_id = msg.header.frame_id
        print(self.map.cell_at(0,1))

    def publish_pose(self):
        """Example of publishing an arrow, without orientation."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame_id
        
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 1.0
        quaternion = tf_transformations.quaternion_from_euler(np.pi/2, 0, 0)
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]

        self.pose_pub.publish(pose_msg)

    def get_transformation(self, start_frame, target_frame):
        """Get transformation between two frames."""
        try:
            while not self.tf_buffer.can_transform(target_frame, start_frame, self.get_clock().now()):
                print("waiting...")
                rclpy.spin_once(self)
            tf_msg = self.tf_buffer.lookup_transform(target_frame, start_frame, self.get_clock().now())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')
            return
    
        self.get_logger().info(f'Received tf message: {tf_msg}')   
        translation = tf_msg.transform.translation
        quaternion = tf_msg.transform.rotation

        t = tf_transformations.translation_matrix([translation.x, translation.y, translation.z])
        R = tf_transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        T = t.dot(R)
        T = np.round(T, decimals=1) # rounding minimizes noise 
        
        return T
    

    # relative to odom rf
    def get_currentloc(self):
        map_T2_bl = self.get_transformation(TF_BASE_LINK, TF_MAP)
        print("map_T2_bl:\n", map_T2_bl)
        bl_p = np.array([0, 0, 0, 1])
        map_p = map_T2_bl.dot(bl_p.transpose())

        return tuple(map_p[:3])

    def treesearch(self, start, end, stack):  # with history         
        # def get_quaternion(pose1, pose2): 
        #     dx = pose1.x -  pose2.x 
        #     dy = pose1.y - pose2.y

        #     yaw = 

        maxX, maxY = self.map.grid.shape

        visited = set(start)
        prev = {start:None}
        frontier = [start]

        while len(frontier) > 0: 
            if stack: 
                leaf = frontier.pop()
            if not stack: 
                leaf = frontier.pop(0)
            print("leaf: ", leaf)
            visited.add(leaf)

            if leaf == end: 
                node = end 
                path = []
                while node != None: 
                    path.append(node)
                    node = prev[node]
                path.reverse()
                return path

            # add neighbors (decided 4 arbitrarily, not 8)
            x, y, z = leaf 
            neighbors = [(x+1,y,z), (x-1,y,z), (x,y+1,z), (x,y-1,z)]  # dfs order - down, up, left, right
            for n in neighbors: 
                x, y, z = n 
                if 0 <= x <= maxX and 0 <= y <= maxY and n not in visited: 
                    print("   neighbor: ", n)
                    prev[n] = leaf
                    frontier.append(n)
                    if n not in visited: 
                        visited.add(n)
        return None 

    def bfs(self, start, end): 
        self.treesearch(start, end, False)

    def dfs(self, start, end): 
        self.treesearch(start, end, True)

def main(args=None):
    # 1st. initialization of node.
    rclpy.init(args=args)

    p = Plan()

    # wait until the map callback has run
    print("Waiting for map... \n(run ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \"{ map_url: 'pa3/maze.yml' }\" in another terminal)")
    
    while p.map is None:
        rclpy.spin_once(p)

    while rclpy.ok():
        p.publish_pose()

        print("this program runs bfs/dfs from the current location to a location of your choosing.")
        while True:
            algo = input("do you want bfs or dfs? ").strip().lower()
            if algo in ["bfs", "dfs"]:
                break
            print("invalid input - enter 'bfs' or 'dfs'")
        
        while True: 
            end_coords = input("Input the goal coordinates (format: x,y): ").strip()
            try:
                end = tuple(map(int, end_coords.split(','))) + (0,)
                print(end)
                if len(end) == 3:
                    if not p.map.inGrid(end[0], end[1]):
                        print("invalid input - that value isn't in the grid") 
                        continue 
                    break
                else:
                    print("invalid input - enter exactly two numbers separated by a comma")
            except ValueError:
                print("invalid input - enter valid integers separated by a comma")
        
        start = p.get_currentloc()

        if algo == "bfs": 
            p.bfs(start, end)
        else: 
            p.dfs(start, end)

        rclpy.spin_once(p)
    # except KeyboardInterrupt:
    #     print("SOMETHING")
    #     pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
