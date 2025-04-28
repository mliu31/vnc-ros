#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Megan Liu
# Date: 4/25/25
# PA2: Wall Following

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number
from enum import Enum

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from pid import PID # PID class implemented in class 

from std_srvs.srv import SetBool # service type 

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_SERVICE_NAME = 'on_off'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 0.5 # m/s
MAX_ANGULAR = 7 # rad/s

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
# Note: these angles are with respect to the robot perspective, but needs to be
# converted to match how the laser is mounted.
LASER_ROBOT_OFFSET = math.pi # angle of the laser that is in front of the robot
MIN_SCAN_ANGLE_RAD = -math.pi + LASER_ROBOT_OFFSET
MAX_SCAN_ANGLE_RAD = 0 + LASER_ROBOT_OFFSET

USE_SIM_TIME = True

KP = 1
KI = 0
KD = 2
K = 0 
distance = 1

class fsm(Enum):
    PID_CALC = 1
    MOVE = 2
    STOP = 3

class WallFollow(Node):
    def __init__(self, distance, linear_velocity=LINEAR_VELOCITY, 
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
        node_name="wall_follow", context=None):
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
        self.angular_velocity = 0 
        self.scan_angle = scan_angle

        # Rate at which to operate the while loop.
        self.rate = self.create_rate(FREQUENCY)
        
        # setting up a service
        self._on_off_service = self.create_service(SetBool, f'{node_name}/{DEFAULT_SERVICE_NAME}', self._turn_on_off_callback)

        # fsm variable
        self._fsm = fsm.PID_CALC

        # distance to wall 
        self.wall_distance = float(distance)
        self.current_distance = math.inf

        # pid 
        self.pid = PID(KP, KI, KD, K)

        # prev time for dt 
        self.start_time = 0 # first LS time in simulation time 
        self.prev_time = 0 
        self.curr_time = 0
             
    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = float(linear_vel)
        twist_msg.angular.z = float(angular_vel)
        self._cmd_pub.publish(twist_msg)

    def move_dt(self, linear_vel, angular_vel, dt): 
        print(f"   Moving {linear_vel} m/s linear velocity, {angular_vel} rad/s angular velocity for ", dt, " seconds")

        secs = dt
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
            self.move(linear_vel, angular_vel) 

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _turn_on_off_callback(self, req, resp):
        if not req.data:
            self._fsm = fsm.STOP
            self.stop()
            resp.success = True
            resp.message = "Robot stopped"
        else:
            if self._fsm == fsm.STOP:
                self._fsm = fsm.MOVE
                resp.success = True
                resp.message = "Robot activated"
            else:
                resp.success = False
                resp.message = "Robot already moving"
        
        return resp

    # ensure val is between min and max
    def clamp(self, val, min_val, max_val): 
        return max(min_val, min(max_val, val))

    def _laser_callback(self, msg):
        print("[LASER]")
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle, 
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...
        if self._fsm == fsm.PID_CALC or self._fsm == fsm.MOVE:
            # Find the minimum range value between min_scan_angle and
            # max_scan_angle
            # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            # Note2: Note that the laser is rotated of 180 degrees, thus the appropriate angles and indices
            # should be found.
            # Please double check the LaserScan message https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######
            
            # it assumes that scan_angle[0] is < scan_angle[1]
            # it also assumes that angle increment is positive (so counterclockwise scan)
            # and that the laser offset is either 0 or -180 degrees
            # and that the coverage is of 360 degrees
            min_index = int(((self.scan_angle[0]) - msg.angle_min) / msg.angle_increment)
            max_index = int(((self.scan_angle[1]) - msg.angle_min) / msg.angle_increment)

            rmin = math.inf
            for i in range(min_index, max_index+1):
                if msg.range_min <= msg.ranges[i] <= msg.range_max:
                    rmin = min(rmin, msg.ranges[i])
            
            self.current_distance = rmin
            print("    wall dist: ", self.current_distance)

            if self.start_time == 0: 
                self.start_time  = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)  # time since start of simulation, not since program start
                self.prev_time = self.start_time
                self.curr_time = self.start_time
                return 

            self.prev_time = self.curr_time 
            self.curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)

            self._fsm = fsm.PID_CALC
            ####### ANSWER CODE END #######

    def spin(self):
        while rclpy.ok():
            print("[SPIN]")
            # Keep looping until user presses Ctrl+C
            # if fsm == move, move 
            # if fsm == pid calculate, calculate, fsm = move 
            # if fsm == stop, stop 

            rclpy.spin_once(self)

            dt = self.curr_time - self.prev_time  
            if dt == 0: continue  # on laserscan initialization 

            if self._fsm == fsm.MOVE:
                self.move_dt(self.linear_velocity, self.angular_velocity, dt)
            elif self._fsm == fsm.PID_CALC:
                err = self.wall_distance - self.current_distance                
                self.angular_velocity = self.clamp(self.pid.step(err, dt), -MAX_ANGULAR, MAX_ANGULAR)
                self._fsm = fsm.MOVE
            else: 
                self._fsm = fsm.STOP
            
            ####### ANSWER CODE END #######      

def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args)

    wall_follow = WallFollow(distance)

    interrupted = False

    # Robot random walks.
    try:
        wall_follow.spin()
    except KeyboardInterrupt:
        interrupted = True
        wall_follow.get_logger().error("ROS node interrupted.")
    finally:
        # workaround to send a stop
        # thread.join()
        if rclpy.ok():
            wall_follow.stop()

    if interrupted:
        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        wall_follow = WallFollow(distance, node_name="random_walk_end", context=new_context)
        wall_follow.get_logger().error("ROS node interrupted.")
        wall_follow.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()
