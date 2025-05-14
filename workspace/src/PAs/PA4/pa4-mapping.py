#!/usr/bin/env python

# Author: Megan Liu
# PA Template
# Spring 2025 CS81 Robotics 

import rclpy # module for ROS APIs
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
# http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html
from geometry_msgs.msg import Pose

import tf_transformations
import tf2_ros # library for transformations.
from tf2_ros import TransformException
from tf2_ros import StaticTransformBroadcaster

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import LaserScan # message type for scan

import math
import numpy as np

NODE_NAME = "mapper"

MAP_TOPIC = "map"
OCCGRID_TOPIC = 'occupancy_grid'
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel' # move 
DEFAULT_SCAN_TOPIC = 'scan' # laser 

TF_BASE_LINK = 'base_link'
TF_ODOM = 'odom'
TF_MAP = 'map'

MAP_FRAME_ID = "map"

USE_SIM_TIME = True

LINEAR_VELOCITY = 10 # m/s
ANGULAR_VELOCITY = math.pi/3 # rad/s

RESOLUTION = 0.01
INITIAL_SIZE = 1001
CELL_UNKNOWN = -1 
CELL_OCCUPIED = 100
CELL_FREE = 0    

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution, origin):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.resolution = resolution
        self.height = self.grid.shape[0]
        self.width  = self.grid.shape[1]
        self.origin = origin

class Mapper(Node):
    def __init__(self, map_frame_id=MAP_FRAME_ID, node_name=NODE_NAME, context=None):
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL, 
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # Setting up transformation listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.map_frame_id = map_frame_id

        # occupancy grid 
        self.occgrid_pub = self.create_publisher(OccupancyGrid, MAP_TOPIC, 10)
        self.sub = self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_callback, 1)
        self.occgrid_frame_id = None
        self.grid = None 
        
        # lidar sensor 
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)

    def map_callback(self, msg):
        print("[MAP CALLBACK]")
        origin = msg.info.origin.position.x, msg.info.origin.position.y  # meters rel to odom
        self.grid = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, origin)
        self.occgrid_frame_id = msg.header.frame_id

    def m_to_cell(self,x, y): # m to pixels
        # column index
        c = x // self.grid.resolution
        # row index
        r = y // self.grid.resolution
        return r,c

    def convert_bl_to_odom_loc(self, coords): # m rel to odom
            odom_T2_bl, _ = self.get_transformation(TF_BASE_LINK, TF_ODOM)

            # print("odom_T2_bl: \n", odom_T2_bl)
            if coords == []: 
                odom_p = np.array([0, 0, 0, 1])
            else: 
                odom_p = np.append(np.array(coords), np.array([0, 1]))

            odom_p = odom_T2_bl.dot(odom_p.transpose())

            return odom_p[:2].tolist()

    def bresenham(self, coord1, coord2, last_pt_obstacle): # in pixels 
            pts_to_val = {} 

            x1, y1 = coord1 
            x2, y2 = coord2  

            dx = abs(x2-x1) 
            dy = abs(y2-y1) 

            x = x1 
            y = y1
            steep = dy > dx 
                
            eps = 0 
            if not steep: 
                while (x < x2 and x2 > x1) or (x > x2 and x2 < x1): 
                    pts_to_val[(x,y)] = 0
                    eps += dy 
                    if 2*eps > dx: 
                        y = y + (1 if y2 > y1 else -1)
                        eps -= dx 
                    
                    x = x + (1 if x2 > x1 else -1)
                
                if last_pt_obstacle: 
                    pts_to_val[(x,y)] = 100 
            else: 
                while (y < y2 and y2 > y1) or (y > y2 and y2 < y1): 
                    pts_to_val[(x,y)] = 0
                    eps += dx 
                    if 2*eps > dy: 
                        x = x + (1 if x2 > x1 else -1)
                        eps -= dy
                    
                    y = y + (1 if y2 > y1 else -1)
                
                if last_pt_obstacle: 
                    pts_to_val[(x,y)] = 100 

            return pts_to_val    

    def update_occ_grid(self, ranges, min_range, max_range, angle_incr, measure_loc_odom_m): 
        print("[UPDATE OCCUPANCY GRID]")

        def idx_to_angle(idx): 
            # angle relative to LS ranges from -pi --> pi (idx 0 to 1600) 
            angle_ls = -math.pi + angle_incr * idx

            # angle relative to base link = LS angle + pi 
            # ranges from 0 to 2pi
            angle_bl = angle_ls + math.pi

            # angle rel to x-axis
            angle_xaxis = (angle_bl + math.pi/2) % (2*math.pi)

            return angle_xaxis
                
        def publish_updated_grid(coords_to_val_dict): 
            print("  publishing updated grid")
            # print(coords_to_val_dict)

            # min of new and existing coords 
            min_x = int(min(min(coords_to_val_dict.keys(), key=lambda k: k[0])[0], 0))
            min_y = int(min(min(coords_to_val_dict.keys(), key=lambda k: k[1])[1], 0))
            max_x = int(max(max(coords_to_val_dict.keys(), key=lambda k: k[0])[0], self.grid.width-1))
            max_y = int(max(max(coords_to_val_dict.keys(), key=lambda k: k[1])[1], self.grid.height-1))

            # min/max vals are indices
            height = max_y - min_y + 1 
            width = max_x - min_x + 1 
            print("    shape: ", self.grid.grid.shape, " --> ", height, width)


            # create new grid based on these dimensions 
            data = np.full((int(height), int(width)), -1)

            # insert prev vals 
            prev_origin = self.grid.origin
            prev_origin_indices = (int(abs(min_x)), int(abs(min_y)))
            prev_width = int(self.grid.width)
            prev_height = int(self.grid.height)

            # updated origin
            ox_new = min_x * self.grid.resolution
            oy_new = min_y * self.grid.resolution
            print("    origin: ", prev_origin, "m, ", prev_origin_indices, " cell in new grid --> ", ox_new, oy_new, "m")

            print(prev_origin_indices[1], prev_height)

            data[prev_origin_indices[1]:prev_height + prev_origin_indices[1], prev_origin_indices[0]:prev_width + prev_origin_indices[0]] = self.grid.grid

            # insert new vals 
            for key,value in coords_to_val_dict.items():
                x,y = int(key[0]), int(key[1]) 
                # print("    x,y,val: ", x,y, value)
                data[y-min_y,x-min_x] = value 
            
            # create new occupancy grid msg 
            og_msg = OccupancyGrid()

            # header 
            og_msg.header.stamp = self.get_clock().now().to_msg()
            og_msg.header.frame_id = self.map_frame_id

            # metadata 
            og_msg.info.width = width # cells 
            og_msg.info.height = height # cells 
            og_msg.info.resolution =  self.grid.resolution # m/cell 

            # update origin (in m rel to odom)
            q = self.make_quat(0)
            origin_posemsg = self.create_posemsg(ox_new, oy_new,0,q)
            og_msg.info.origin = origin_posemsg # origin of map [m, m, rad] - real world pose of cell (0,0) in map

            og_msg.data = data.flatten().tolist()

            # publish msg
            self.occgrid_pub.publish(og_msg)

        pts_to_val = {}

        loc_at_measurement_cell = self.m_to_cell(measure_loc_odom_m[0], measure_loc_odom_m[1])
        # print(self.loc_at_measurement, loc_at_measurement_cells)
        
        for i, r in enumerate(ranges):
            if r == math.inf: 
                r = 10
                hit_obst = False
            else: 
                hit_obst = True 

            if min_range <= r <= max_range: 
                r = r // self.grid.resolution # m to cells 
                angle = idx_to_angle(i)
                # print("  angle wrt x-axis" , angle, " r:", r)
                
                # rel to cartesian coords 
                x = r * math.cos(angle) 
                y = r * math.sin(angle)
                detectedloc_cell = [x,y] # rel to bl
                # print("  detected_loc_cell: ", detected_loc_cell)

                detected_loc_odom_cell = self.convert_bl_to_odom_loc(detectedloc_cell)
                # detected_loc_odom_meter = self.m_to_cell(detected_loc_odom_cell[0], detected_loc_odom_cell[1])
                # print("  odom coords: ", detected_loc_odom)

                # print("    ", loc_at_measurement_cells, detected_loc_odom)
                oneangle_pts_to_val = self.bresenham(loc_at_measurement_cell, detected_loc_odom_cell, hit_obst)
                # print("  ", oneangle_pts_to_val)

                pts_to_val.update(oneangle_pts_to_val)
        
        if pts_to_val: 
            # print(pts_to_val)
            publish_updated_grid(pts_to_val)

    def _laser_callback(self, laserscan_msg): 
        print("[LASER CALLBACK]")
        
        ranges = laserscan_msg.ranges
        min_range = laserscan_msg.range_min
        max_range = laserscan_msg.range_max 
        angle_incr = laserscan_msg.angle_increment

        currloc_odom_m = tuple(self.convert_bl_to_odom_loc([]))

        self.update_occ_grid(ranges, min_range, max_range, angle_incr, currloc_odom_m)
        
    def get_transformation(self, start_frame, target_frame):
        """Get transformation between two frames."""
        try:
            while not self.tf_buffer.can_transform(target_frame, start_frame, self.get_clock().now()):
                # print("waiting for transform...")
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

    def make_quat(self, yaw):
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def create_posemsg(self, x, y, z, q):
        """Example of publishing an arrow, without orientation."""
        pose_msg = Pose()
        
        pose_msg.position.x = float(x)
        pose_msg.position.y = float(y)
        pose_msg.position.z = float(z)
        pose_msg.orientation.x = float(q.x)
        pose_msg.orientation.y = float(q.y)
        pose_msg.orientation.z = float(q.z)
        pose_msg.orientation.w = float(q.w)

        return pose_msg

    def init_occgrid(self): 
        print("[INITIALIZE OCCUPANCY GRID]")
        og_msg = OccupancyGrid()
        # header 
        og_msg.header.stamp = self.get_clock().now().to_msg()
        og_msg.header.frame_id = self.map_frame_id

        # metadata 
        og_msg.info.width = INITIAL_SIZE # cells 
        og_msg.info.height = INITIAL_SIZE # cells 
        og_msg.info.resolution =  RESOLUTION # m/cell 

        # origin = odom rf origin 
        q = self.make_quat(0)
        origin_posemsg = self.create_posemsg(0,0,0,q)
        og_msg.info.origin = origin_posemsg # origin of map [m, m, rad] - real world pose of cell (0,0) in map

        # map data (list of -1, 0, 100 values = unknown, empty, occupied, respectively)
        data = [-1] * (og_msg.info.width * og_msg.info.height)

        og_msg.data = data
        self.occgrid_pub.publish(og_msg)

    def spin(self):
        self.init_occgrid()
        while rclpy.ok():
            print("[SPINNING]")
            rclpy.spin_once(self)

class Broadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_map_broadcaster')
        self.get_logger().info('Initializing static transform broadcaster')
        self.br = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.frame_id    = TF_ODOM
        t.child_frame_id     = TF_MAP
        t.header.stamp       = self.get_clock().now().to_msg()
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.rotation.w    = 1.0 # identity rot
        self.br.sendTransform(t)

def main():
    rclpy.init()

    mapper = Mapper()
    Broadcaster()  # publishes static (identity) transform between odom and map 

    mapper.spin()

    rclpy.shutdown()

if __name__ == "__main__":
    main()