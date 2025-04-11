#!/usr/bin/env python

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist # message type for velocity command.
from sensor_msgs.msg import LaserScan # message type for laser measurement.

import tf2_ros # library for transformations.
from tf2_ros import TransformException

import tf_transformations

import numpy as np

# Constants.
FREQUENCY = 10 #Hz.
VELOCITY = 0.2 #m/s
DURATION = 5 #s how long the message should be published.
USE_SIM_TIME = True
NODE_NAME = "test_tf"
DEFAULT_SCAN_TOPIC = 'scan'
TF_BASE_LINK = 'base_link'
TF_LASER_LINK = 'laser'

class Test(Node):
    """Class example for a ROS node."""
    def __init__(self):
        """Constructor."""
        super().__init__(node_name=NODE_NAME)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # 2nd. setting up publishers/subscribers.
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)

        # Setting up transformation listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _laser_callback(self, laserscan_msg):
        """Callback function."""

        """ Transforming points in reference frames."""
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
        rpy = tf_transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # TODO: transform lidar points in base_link
        # perform matrix mult based on euler angles 
        def rotation_x(roll): 
            return np.array([
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)]
            ])
        
        def rotation_y(pitch):
            return np.array([
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)]
            ])
        
        def rotation_z(yaw):
            return np.array([
                [np.cos(yaw), -np.sin(yaw), 0],
                [np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1]
            ])
        
        roll = rpy[0]
        pitch = rpy[1]
        yaw = rpy[2]

        bl_t_bll = np.array([[translation.x], [translation.y], [translation.z]])
        bl_R_bll = rotation_x(roll).dot(rotation_y(pitch)).dot(rotation_z(yaw))

        def homogenous_matrix(R, t): 
            return np.vstack( 
                (np.hstack((R, t)), [0, 0, 0, 1])
            )
        
        bl_T_l = homogenous_matrix(bl_R_bll, bl_t_bll)
        

        # inverse 
        def inverse_T(T): 
            Rtranspose = np.transpose(T[:3, :3])
            tinverse = -Rtranspose.dot(T[:3, 3:])

            return homogenous_matrix(Rtranspose, tinverse)
        
        l_T_bl = inverse_T(bl_T_l)

        # Equivalent code
        # t = tf_transformations.translation_matrix([translation.x, translation.y, translation.z])
        # R = tf_transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # bl_T2_l = t.dot(R)
        ##

        self.get_logger().info(f'got: {tf_msg}\n{[roll, pitch, yaw]}\n{R}\n{bl_R_bll}\n{bl_T_l}\n{bl_T2_l}\n{l_T_bl}')

        for i, r in enumerate(laserscan_msg.ranges): 
            angle = laserscan_msg.angle_min + laserscan_msg.angle_increment * i 
            x = r*np.cos(angle)
            y = r*np.sin(angle) 

            l_p = np.array([x, y, 0, 1])
            bl_p = bl_T_l.dot(l_p.transpose())

            if i == 0: 
                print(f"1: l_p: {l_p}\n bl_p {bl_p}")
                break

        ### END TODO

def main():
    """main function."""
    rclpy.init()

    node = Test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
