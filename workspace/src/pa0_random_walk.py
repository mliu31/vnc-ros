#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Megan Liu
# Class: COSC81.01 Principles of Robot Design and Programming
# Instructor: Alberto Quattrini Li
# Term: Spring 2025 (4/4/2025)
# Assignment: PA0 - Random Walk
# Description: This code implements a random walk behavior for a robot using ROS2. The robot moves forward until it detects an obstacle within a certain distance, at which point it rotates a random angle [-pi, pi]  before continuing to move forward.

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

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 0.3 # m/s
ANGULAR_VELOCITY = math.pi # rad/s

# Threshold of minimum clearance distance (TODO: feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.3 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
# MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi
# MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi

USE_SIM_TIME = True

class RandomWalk(Node):
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
        node_name="random_walk", context=None):
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
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle

        # Rate at which to operate the while loop.
        self.rate = self.create_rate(FREQUENCY)
        
        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.

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

    def _laser_callback(self, msg):
        print("[LASER] flag: ", self._close_obstacle)

        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle, 
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...

        if not self._close_obstacle:
            print("   checking for obstacles")
            # Find the minimum range value between min_scan_angle and max_scan_angle
            # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Please double check the LaserScan message https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html

            ####### TODO: ANSWER CODE BEGIN #######
            # angles relative to LaserScanner ref frame
            start_scan_angle = -math.pi - self.scan_angle[0] # -pi/2 
            end_scan_angle = math.pi - self.scan_angle[1] # pi/2 

            print(start_scan_angle, end_scan_angle)

            # angular difference between LaserScanner angle_min and start_scan_angle
            # and between LaserScanner angle_max and end_scan_angle
            start_diff = start_scan_angle - msg.angle_min
            end_diff = msg.angle_max - end_scan_angle

            if start_diff < 0 and end_diff < 0: # no valid angles in ranges array
                print("   invalid scanning angles, no distances checked")
                return
            
            minidx = int((start_diff) / msg.angle_increment)
            maxidx = int((end_diff) / msg.angle_increment)

            mindist = min(msg.ranges[:minidx] + msg.ranges[-maxidx:])
        
            if mindist < MIN_THRESHOLD_DISTANCE: 
                self._close_obstacle = True 
                print("      found obstacle; set flag to true")
            ####### ANSWER CODE END #######

    def spin(self):
        while rclpy.ok():
            print("[SPIN] flag: ", self._close_obstacle)

            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random angle (use random.uniform() to generate a random value)
            # after which the flag is set again to False.
            # Use the function move to publish velocities already implemented,
            # passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######
            if self._close_obstacle == False: 
                self.move(self.linear_velocity, 0.0)

                print("   no obstacle, moving forward")
            else: 
                
                rand_ang = random.uniform(-math.pi, math.pi)
                print("   obstacle detected! rotating ", rand_ang, " rad; set flag to false")
                secs = abs(rand_ang) / ANGULAR_VELOCITY
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
                    if rand_ang > 0: 
                        self.move(0.0, -self.angular_velocity)  # counterclockwise 
                    else: 
                        self.move(0.0, self.angular_velocity)  # clockwise
                
                print("sleeping ---------------------------------------------------------------------------------------------------------------")
                time.sleep(10)
                self._close_obstacle = False 
            ####### ANSWER CODE END #######

            # operating at the set frequency
            # https://robotics.stackexchange.com/questions/96684/rate-and-sleep-function-in-rclpy-library-for-ros2
            rclpy.spin_once(self)
            

        

def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args)

    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    interrupted = False

    # Robot random walks.
    try:
        random_walk.spin()
    except KeyboardInterrupt:
        interrupted = True
        random_walk.get_logger().error("ROS node interrupted.")
    finally:
        # workaround to send a stop
        # thread.join()
        if rclpy.ok():
            random_walk.stop()

    if interrupted:
        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        random_walk = RandomWalk(node_name="random_walk_end", context=new_context)
        random_walk.get_logger().error("ROS node interrupted.")
        random_walk.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()
