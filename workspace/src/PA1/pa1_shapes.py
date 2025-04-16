#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Megan Liu
# Class: COSC81.01 Principles of Robot Design and Programming
# Instructor: Alberto Quattrini Li
# Term: Spring 2025 (4/4/2025)
# Assignment: PA1 - Shape Draw

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
import time 

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from rclpy.duration import Duration # message type for duration

import tf2_ros # library for transformations.
from tf2_ros import TransformException

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

TF_BASE_LINK = 'base_link'
TF_ODOM = 'odom'

# Frequency at which the loop operates
FREQUENCY = 1 #Hz.

# Velocities that will be used 
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/6 # rad/s

# Field of view in radians that is checked in front of the robot 
MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi

USE_SIM_TIME = True

class ShapeDraw(Node): 
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
        node_name="shape_draw", context=None):

        """Constructor."""
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.scan_angle = scan_angle

        # Rate at which to operate the while loop.
        self.rate = self.create_rate(FREQUENCY)

         # Setting up transformation listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def move(self, linear_vel, angular_vel):
            """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
            # Setting velocities.
            twist_msg = Twist()

            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self._cmd_pub.publish(twist_msg)
           
    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    # angles in radians
    def rotate(self, angle):
        print(f"   Rotating {angle} rad, {angle*180/math.pi} degrees")

        secs = abs(angle) / self.angular_velocity
        duration = Duration(seconds=secs)
        start_time = self.get_clock().now()

        # rotate for certain duration 
        while rclpy.ok():
            # print("rotating")
            rclpy.spin_once(self)
            # Log current time information.
            
            # Check if the specified duration has elapsed.
            if self.get_clock().now() - start_time >= duration:
                break
            # Publish the twist message continuously.
            if angle > 0: 
                self.move(0.0, self.angular_velocity)  # counterclockwise 
            else: 
                self.move(0.0, -self.angular_velocity)  # clockwise

    # distance in m 
    def translate(self, distance): 
        print(f"   Moving forward {distance}m")
        
        secs = abs(distance) / self.linear_velocity
        duration = Duration(seconds=secs)
        start_time = self.get_clock().now()

        # rotate for certain duration 
        while rclpy.ok():
            rclpy.spin_once(self)
            # Log current time information.
            
            # Check if the specified duration has elapsed.
            if self.get_clock().now() - start_time >= duration:
                break
            # Publish the twist message continuously.
            self.move(self.linear_velocity, 0.0)  

    def trapezoid(self, r): 
        self.get_logger().info(f"Trapezoid")
        # rotate π/4 radians - trapezoid is perpendicular to base link rf 
        self.rotate(math.pi/2)

        # # move forward r
        self.translate(r)

        # rotate supplement of θ_1 (angle in top left corner)
        θ_1 = math.atan(r/math.sqrt(2) / (r-(r/math.sqrt(2))))
        θ_1_supp = math.pi - θ_1
        self.rotate(-θ_1_supp) # ccw
        
        # move forward hypotenuse 
        hypotenuse = math.sqrt((r/math.sqrt(2))**2 + (r-(r/math.sqrt(2)))**2)
        self.translate(hypotenuse)

        # rotate θ_1 (corresponding angles)
        self.rotate(-θ_1)

        # short side of trapezoid 
        short_side = 2 * r/math.sqrt(2)
        self.translate(short_side)

        # rotate θ_1 
        self.rotate(-θ_1)

        # move forward hypotenuse 
        self.translate(hypotenuse)

        # rotate θ_1_supp
        self.rotate(-θ_1_supp)

        # move forward r
        self.translate(r)

    def semicircle(self, r): 
        rclpy.spin_once(self)
        print(f"   Creating semicircle of radius {r}m")
        
        # l = vΔt =>  Δt = θ/ω
        secs = math.pi * r / self.linear_velocity 
        duration = Duration(seconds=secs)
        start_time = self.get_clock().now()

        # r = l/θ = vΔt/ωΔt => ω = v/r
        w = self.linear_velocity / r

        # rotate for certain duration 
        while rclpy.ok():
            rclpy.spin_once(self)
            # Log current time information.
            
            # Check if the specified duration has elapsed.
            # print(self.get_clock().now() - start_time, duration)
            if self.get_clock().now() - start_time >= duration:
                break
            
            # Publish the twist message continuously.
            self.move(self.linear_velocity, -w)
            # self.move(v, 0.0) 

    def D(self, r): 
        self.get_logger().info(f"D")
        # rotate π/4 radians - D is perpendicular to base link rf 
        self.rotate(math.pi/2)
        
        # move forward r 
        self.translate(r)
        # rotate -π/4 radians
        self.rotate(-math.pi/2)

        # semicircle
        self.semicircle(r)

        # rotate -π/4 radians
        self.rotate(-math.pi/2)

        # move forward r 
        self.translate(r)

    def polygon(self, odom_coords): 
        self.get_logger().info(f"Polygon")

        print(odom_coords)
        
        # get transformation matrix from odom to base_link rf
        try:
            tf_msg = self.tf_buffer.lookup_transform(TF_BASE_LINK, TF_LASER_LINK, laserscan_msg.header.stamp)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform: {ex}')
            return
        self.get_logger().info(
                f'got: {tf_msg}')
        translation = tf_msg.transform.translation
        quaternion = tf_msg.transform.rotation

        

        bl_coords = [] 

    def spin(self):
        while rclpy.ok(): 
            # Keep looping until user presses Ctrl+C
             
            # ask user what kind of shape they want to draw (1) isolesces trapezoid, (2) D, (3) polygon
            # if user presses 1, ask for r then draw 
            # if user presses 2, ask for r then draw 
            # if user presses 3, ask for (x,y) points and enter after each point (once done, press enter) then draw  
            
            
            rclpy.spin_once(self)
       
            self.D(2)

            # coords = [[1, 0], [1, 1], [0, 1]]
            # self.polygon(coords)

            """response = input("What kind of shape do you want to draw?  \n(1) isolesces trapezoid, \n(2) D, \n(3) polygon \n(type 1, 2, or 3)\n")
        
            while response != "1" and response != "2" and response != "3":
                response = input("Invalid response. Type 1, 2, or 3 for... \n(1) isolesces trapezoid, \n(2) D, \n(3) polygon\n")

            match response: 
                case "1":
                    r = input("Enter the radius of the isosceles trapezoid: ")     

                    while not r.isnumeric():  
                        r = input("Invalid response. Try again...\n")
    
                    rclpy.spin_once(self)
                    self.trapezoid(float(r))

                case "2":
                    r = input("Enter the radius of the isosceles trapezoid: ")     

                    while not r.isnumeric():  
                        r = input("Invalid response. Try again...\n")
    
                    rclpy.spin_once(self)
                    self.D(float(r))
                case "3":
                    coords = [] 
                    coord = input("Enter the coordinates of the vertices of polygon (relative to odom rf) in the form of x,y (no space between) and press enter after each point. e.g., 1,3\nWhen done entering, press enter.\n")
                
                    while coord != "": 
                        if coord.count(",") != 1 or not coord.split(",")[0].isnumeric() or not coord.split(",")[1].isnumeric(): 
                            coord = input("Invalid response. Try again...\n")

                        else:
                            coords.append([float(coord.split(",")[0]), float(coord.split(",")[1])])
                            coord = input("Enter next coordinate or press enter to finish: ")
                            
                    rclpy.spin_once(self)
                    self.polygon(coords)"""

       
            break 

            ####### ANSWER CODE END #######

            # operating at the set frequency
            # https://robotics.stackexchange.com/questions/96684/rate-and-sleep-function-in-rclpy-library-for-ros2
            rclpy.spin_once(self)            

def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args)

    # Initialization of the class for the random walk.
    shape_draw = ShapeDraw()

    interrupted = False

    # Robot draws shape 
    try:
        shape_draw.spin()
    except KeyboardInterrupt:
        interrupted = True
        shape_draw.get_logger().error("ROS node interrupted.")
    finally:
        # workaround to send a stop
        # thread.join()
        if rclpy.ok():
            shape_draw.stop()

    if interrupted:
        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        shape_draw = ShapeDraw(node_name="shape_draw_end", context=new_context)
        shape_draw.get_logger().error("ROS node interrupted.")
        shape_draw.stop()
        rclpy.try_shutdown()

if __name__ == "__main__":
    """Run the main function."""
    main()
