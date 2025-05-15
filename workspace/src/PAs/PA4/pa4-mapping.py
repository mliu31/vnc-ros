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
INITIAL_SIZE = 100
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
    
    def get_yaw(self, quat): 
        _, _, yaw = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return yaw

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


    def m_to_cell(self,x, y): # m to pixels
        # column index
        c = x // self.grid.resolution
        # row index
        r = y // self.grid.resolution
        return round(r),round(c)

    def cell_to_m(self, r,c): 
        x = c * self.grid.resolution 
        y = r * self.grid.resolution
        return round(x),round(y)

    def convert_bl_to_odom_loc(self, coords): # m rel to odom
            odom_T2_bl, quaternion = self.get_transformation(TF_BASE_LINK, TF_ODOM)

            # print("odom_T2_bl: \n", odom_T2_bl)
            if coords == []: 
                odom_p = np.array([0, 0, 0, 1])
            else: 
                odom_p = np.append(np.array(coords), np.array([0, 1]))

            odom_p = odom_T2_bl.dot(odom_p.transpose())
            # print(odom_p)

            return odom_p[:2].tolist(), quaternion

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

    def update_occ_grid(self, ranges, min_range, max_range, angle_incr, measure_loc_odom_m, measure_loc_quat): 
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
        
        def process_sensor_data(): 
            print("  processing sensor data")
            pts_to_val = {}

            print("     current location: ", measure_loc_odom_m)
            loc_at_measurement_cell = self.m_to_cell(measure_loc_odom_m[0], measure_loc_odom_m[1])
            
            # process sensor data 
            for i, r in enumerate(ranges):
                if r == math.inf: 
                    r = 10
                    hit_obst = False
                else: 
                    hit_obst = True 

                if min_range <= r <= max_range: 
                    r = r // self.grid.resolution # m to cells 
                    angle_xaxis = idx_to_angle(i)
                    
                    # cartesian cell 
                    x = round(r * math.cos(angle_xaxis))
                    y = round(r * math.sin(angle_xaxis))
                    detectedloc_cartesian_cell = (x,y)

                    yaw = self.get_yaw(measure_loc_quat)
                    xaxis_to_bl_angle = yaw + (math.pi/2)

                    # detectedloc_bl_cell = pass 
                    detected_bl_m = self.cell_to_m(detectedloc_bl_cell[1], detectedloc_bl_cell[0])
                    if i == 0: 
                        print("     angle from x axis, x-axis to bl: ", angle_xaxis, xaxis_to_bl_angle)
                        print("     detectedloc_cartesian_cell", detectedloc_cartesian_cell)
                        # print("     detectedloc_bl_cell", detectedloc_bl_cell)
                        print("     detected bl_m", detected_bl_m)
    
                    detectedloc_odom_m, _ = self.convert_bl_to_odom_loc(detected_bl_m)
                    detectedloc_odom_cell = self.m_to_cell(detectedloc_odom_m[1], detectedloc_odom_m[0])
                    if i == 0: 
                        print("      detectedloc_odom_m: ", detectedloc_odom_m)
                        print("      detectedloc_odom_cell: ", detectedloc_odom_cell)
                    
                    
                    oneangle_pts_to_val = self.bresenham(loc_at_measurement_cell, detectedloc_odom_cell, hit_obst)

                    pts_to_val.update(oneangle_pts_to_val)
            
            if pts_to_val:
                max_y = max(pts_to_val.keys(), key=lambda k: k[1])[1]
                print("              largest y value in pts_to_val: ", max_y)

            return pts_to_val
        
        def update_griddata(cells_odom_to_val): 
            print("  updating grid data")

            ## RESIZE GRID 
            
            # min of new cells (cells in odom)
            minx_new_odom_cell = min(cells_odom_to_val.keys(), key=lambda k: k[0])[0]
            miny_new_odom_cell = min(cells_odom_to_val.keys(), key=lambda k: k[1])[1]
            maxx_new_odom_cell = max(cells_odom_to_val.keys(), key=lambda k: k[0])[0]
            maxy_new_odom_cell = max(cells_odom_to_val.keys(), key=lambda k: k[1])[1]

            # prev origin (m and cell in odom)
            ox_prev_m_odom, oy_prev_m_odom = self.grid.origin
            oy_prev_cell_odom, ox_prev_cell_odom = self.m_to_cell(ox_prev_m_odom, oy_prev_m_odom)

            # min of prev grid and new cells
            prev_width = int(self.grid.width)
            prev_height = int(self.grid.height)
            
            minx_odom_cell = min(minx_new_odom_cell, ox_prev_cell_odom)
            miny_odom_cell = min(miny_new_odom_cell, oy_prev_cell_odom)
            maxx_odom_cell = max(maxx_new_odom_cell, prev_width-1 + ox_prev_cell_odom)
            maxy_odom_cell = max(maxy_new_odom_cell, prev_height-1 + oy_prev_cell_odom)

            height = int(maxy_odom_cell - miny_odom_cell + 1)
            width = int(maxx_odom_cell - minx_odom_cell + 1)

            # create resized grid 
            data = np.full((height, width), -1)
            print("    shape: ", self.grid.grid.shape, " --> ", height, width)

            ## UPDATE ORIGIN 

            # prev origin cell in new grid 
            ox_prev_cell_newgrid, oy_prev_cell_newgrid = (int(ox_prev_cell_odom-minx_odom_cell), int(oy_prev_cell_odom-miny_odom_cell))

            # new origin 
            origin_new_odom_cells = (minx_odom_cell, miny_odom_cell)
            origin_new_odom_m = self.cell_to_m(origin_new_odom_cells[1], origin_new_odom_cells[0])

            print("    prev origin: ", ox_prev_m_odom, oy_prev_m_odom, "m odom, ", ox_prev_cell_odom, oy_prev_cell_odom,  "cell in odom --> ", ox_prev_cell_newgrid, oy_prev_cell_newgrid, " cell in new grid")
            print("    new origin: ", origin_new_odom_m[0], origin_new_odom_m[1], "m odom, ", origin_new_odom_cells, " cell in odom")
            
            ## UPDATE DATA IN NEW GRID

            # insert prev grid
            data[oy_prev_cell_newgrid:prev_height + oy_prev_cell_newgrid, ox_prev_cell_newgrid:prev_width + ox_prev_cell_newgrid] = self.grid.grid

            # insert new data 
            for key,value in cells_odom_to_val.items():
                x,y = int(key[0]-origin_new_odom_cells[1]), int(key[1]-origin_new_odom_cells[0]) 
                data[y,x] = value 

            return data, width, height, origin_new_odom_m 
                
        def publish_updated_grid(data, width, height, origin): 
            print("  publishing updated grid")

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
            ox, oy = origin
            origin_posemsg = self.create_posemsg(ox, oy, 0,q)
            og_msg.info.origin = origin_posemsg # origin of map [m, m, rad] - real world pose of cell (0,0) in map

            og_msg.data = data.flatten().tolist()

            # publish msg
            self.occgrid_pub.publish(og_msg)

        cells_odom_to_val = process_sensor_data()
        data, width, height, origin = update_griddata(cells_odom_to_val)
        publish_updated_grid(data, width, height, origin)

    def _laser_callback(self, laserscan_msg): 
        print("[LASER CALLBACK]")
        
        ranges = laserscan_msg.ranges
        min_range = laserscan_msg.range_min
        max_range = laserscan_msg.range_max 
        angle_incr = laserscan_msg.angle_increment

        currloc_odom_m, quat = tuple(self.convert_bl_to_odom_loc([]))

        self.update_occ_grid(ranges, min_range, max_range, angle_incr, currloc_odom_m, quat)
    
    def map_callback(self, msg):
        print("[MAP CALLBACK]")
        origin = msg.info.origin.position.x, msg.info.origin.position.y  # meters rel to odom
        self.grid = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution, origin)
        self.occgrid_frame_id = msg.header.frame_id

    def init_occgrid(self): 
        print("[INITIALIZE OCCUPANCY GRID]")
        og_msg = OccupancyGrid()
        # header 
        og_msg.header.stamp = self.get_clock().now().to_msg()
        og_msg.header.frame_id = self.map_frame_id

        # metadata 
        og_msg.info.width = 500 # cells 
        og_msg.info.height = 1000 # cells 
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