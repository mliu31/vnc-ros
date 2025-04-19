#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: AQL
# Date: AQL

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

from std_srvs.srv import SetBool # service type 

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_SERVICE_NAME = 'on_off'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 1.0 # m/s
ANGULAR_VELOCITY = math.pi/4 # rad/s

# Threshold of minimum clearance distance (TODO: feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
# Note: these angles are with respect to the robot perspective, but needs to be
# converted to match how the laser is mounted.
LASER_ROBOT_OFFSET = -math.pi # angle of the laser that is in front of the robot
MIN_SCAN_ANGLE_RAD = -10.0 / 180 * math.pi + LASER_ROBOT_OFFSET
MAX_SCAN_ANGLE_RAD = +10.0 / 180 * math.pi + LASER_ROBOT_OFFSET

RANDOM_ROTATION_LEFT = -math.pi
RANDOM_ROTATION_RIGHT = math.pi

USE_SIM_TIME = True

class fsm(Enum):
    WAITING_FOR_LASER = 0
    MOVE_FORWARD = 1
    ROTATE_CALC = 2
    ROTATE = 3
    STOP = 4

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
        
        # setting up a service
        self._on_off_service = self.create_service(SetBool, f'{node_name}/{DEFAULT_SERVICE_NAME}', self._turn_on_off_callback)

        # fsm variable.
        self._fsm = fsm.STOP

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

    def _turn_on_off_callback(self, req, resp):
        if not req.data:
            self._fsm = fsm.STOP
            self.stop()
            resp.success = True
            resp.message = "Robot stopped"
        else:
            if self._fsm == fsm.STOP:
                self._fsm = fsm.WAITING_FOR_LASER
                resp.success = True
                resp.message = "Robot activated"
            else:
                resp.success = False
                resp.message = "Robot already moving"
        
        return resp

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle, 
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...

        if self._fsm == fsm.MOVE_FORWARD or self._fsm == fsm.WAITING_FOR_LASER:
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

            # print(f"{min_index} {max_index}")
            for i in range(min_index, max_index+1):
                if msg.range_min <= msg.ranges[i] <= msg.range_max and msg.ranges[i] < self.min_threshold_distance:
                    self._fsm = fsm.ROTATE_CALC
                    break
            if self._fsm != fsm.ROTATE_CALC:
                self._fsm = fsm.MOVE_FORWARD
            ####### ANSWER CODE END #######

    def spin(self):
        while rclpy.ok():
            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random angle (use random.uniform() to generate a random value)
            # after which the flag is set again to False.
            # Use the function move to publish velocities already implemented,
            # passing the default velocities saved in the corresponding class members.

            # operating at the set frequency
            # https://robotics.stackexchange.com/questions/96684/rate-and-sleep-function-in-rclpy-library-for-ros2
            rclpy.spin_once(self)

            ####### TODO: ANSWER CODE BEGIN #######
            if self._fsm == fsm.MOVE_FORWARD:
                self.move(self.linear_velocity, 0.0)
            else:
                if self._fsm == fsm.ROTATE_CALC:
                    random_angle = random.uniform(RANDOM_ROTATION_LEFT, RANDOM_ROTATION_RIGHT)

                    duration = Duration(seconds=abs(random_angle/self.angular_velocity))
                    self.current_angular_velocity = math.copysign(1, random_angle) * self.angular_velocity
                    self._fsm = fsm.ROTATE
                    start_time = self.get_clock().now()
                if self._fsm == fsm.ROTATE:
                    # Check if traveled of given distance based on time.
                    if self.get_clock().now() - start_time >= duration:
                        self._fsm = fsm.WAITING_FOR_LASER
                    else:
                        # Publish message.
                        self.move(0.0, self.current_angular_velocity)
            
            ####### ANSWER CODE END #######


            

        

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
