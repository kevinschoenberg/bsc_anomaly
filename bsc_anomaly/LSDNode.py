#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class LSDNode(Node):
    """
    _
    """
    def __init__(self):
        super().__init__('LSD')
        
        self.drive_ack = self.create_publisher(
            AckermannDriveStamped, 
            '/drive', 
            10)
        
        self.keysub = self.create_subscription(
            String, 
            '/resetpos', 
            self.key_pressed_call_back, 
            10)
        
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
    def key_pressed_call_back(self,data):
        if (data.data == 'k'):
            print("v")
            # speed 0
            self.drive_ack_pub(0.0)
            # set inital pos
            self.set_initial_pose()
            #
            time.sleep(1.)
            # speed 5
            self.drive_ack_pub(5.0)


    def odom_callback(self, msg):
        #self.get_logger().info('%s' % msg.twist.twist)
        print(msg.twist.twist.linear)

    def drive_ack_pub(self, speed):
        vel = AckermannDriveStamped()
        vel.drive.speed = speed
        self.drive_ack.publish(vel)

    def set_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = -78.0
        pose.pose.pose.position.y = -28.0
        self.pose_pub.publish(pose)
    
    def __del__(self):
        # body of destructor
        self.drive_ack_pub(0.0)


def main(args=None):
    rclpy.init(args=args)
    LSDN = LSDNode()
    #LSDN.drive_ack_pub(5.0)

    rclpy.spin(LSDN)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    LSDN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()