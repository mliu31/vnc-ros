#!/usr/bin/env python
# The line above is important so that this file is interpreted with Python when running it.
# Author: Alberto Quattrini Li
# Date: 2025-03-31

import rclpy  # module for ROS APIs
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.duration import Duration
from geometry_msgs.msg import Twist  # message type
from sensor_msgs.msg import LaserScan  # message type for scan

# Constants
FREQUENCY = 10           # Hz.
LINEAR_VELOCITY = 1.0    # m/s
DURATION = 5.0           # s, how long the message should be published.
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'
USE_SIM_TIME = True


class GoForward(Node):
    def __init__(self, linear_velocity=LINEAR_VELOCITY, node_name="go_forward", context=None):
        """Constructor."""
        super().__init__(node_name, context=context)
        
        # Workaround to not use roslaunch by setting the 'use_sim_time' parameter.
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])
        
        # Setting up publishers and subscribers.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)
        
        # Other variables.
        self.linear_velocity = linear_velocity  # Constant linear velocity.
        # Rate at which to operate the while loop.
        self.rate = self.create_rate(FREQUENCY)

    def move_forward(self, duration):
        """Function to move forward for a given duration."""
        # Setting the forward velocity.
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity
        duration = Duration(seconds=duration)
        rclpy.spin_once(self)
        start_time = self.get_clock().now()

        # Loop until the duration has elapsed.
        while rclpy.ok():
            rclpy.spin_once(self)
            # Log current time information.
            self.get_logger().info(f"Start time: {start_time}, Current time: {self.get_clock().now()}, Duration: {duration}")
            
            # Check if the specified duration has elapsed.
            if self.get_clock().now() - start_time >= duration:
                break
            # Publish the twist message continuously.
            self._cmd_pub.publish(twist_msg)
        
        # After moving the required time, stop the robot.
        self.stop()

    def stop(self):
        """Stop the robot by publishing zero velocities."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Process the laser message."""
        # Log the incoming laser scan data.
        self.get_logger().info(f"LaserScan: {msg}")


def main(args=None):
    # print(DURATION)
    rclpy.init(args=args)
    go_forward = GoForward()
    go_forward.move_forward(DURATION)
    rclpy.shutdown()


if __name__ == "__main__":
    main()