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
import time


class LSDNode(Node):
    """
    _
    """
    def __init__(self):
        super().__init__('LSD')

        self.drive = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        
        self.drive_ack = self.create_publisher(
            AckermannDriveStamped, 
            '/drive', 
            10)

    def listener_callback(self, msg):
        #self.get_logger().info('')
        a = 1 + 2

    def odom_callback(self, msg):
        #self.get_logger().info('%s' % msg.twist.twist)
        print(msg.twist.twist.linear)
    
    def drive_pub(self, speed):
        vel = Twist()
        vel.linear.x = speed
        self.drive.publish(vel)

    def drive_ack_pub(self, speed):
        vel = AckermannDriveStamped()
        vel.drive.speed = speed
        self.drive_ack.publish(vel)

        
        


def main(args=None):
    rclpy.init(args=args)
    LSDN = LSDNode()
    
    LSDN.drive_ack_pub(5.0)

    rclpy.spin(LSDN)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    LSDN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()