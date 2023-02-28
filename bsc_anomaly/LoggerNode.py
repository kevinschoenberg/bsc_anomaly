#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import os
from os.path import expanduser
from time import gmtime, strftime

import csv
import time



class LoggerNode(Node):
    """
    _
    """
    def __init__(self):
        super().__init__('Logger')

        home = expanduser('~')
        #file = open(strftime(home+'/sim_ws/our_test_logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        print(home)
        file = open(home+'/sim_ws/countries.csv', 'w')

        self.odom = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.save_waypoint, 
            10)
        self.odom

    

def save_waypoint(data):
    quaternion = np.array([data.pose.pose.orientation.x, 
                           data.pose.pose.orientation.y, 
                           data.pose.pose.orientation.z, 
                           data.pose.pose.orientation.w])

    euler = tf.transformations.euler_from_quaternion(quaternion)
    speed = LA.norm(np.array([data.twist.twist.linear.x, 
                              data.twist.twist.linear.y, 
                              data.twist.twist.linear.z]),2)
    if data.twist.twist.linear.x>0.:
        print (data.twist.twist.linear.x)

    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                     data.pose.pose.position.y,
                                     euler[2],
                                     speed))

        


def main(args=None):
    rclpy.init(args=args)
    LN = LoggerNode()

    rclpy.spin(LN)

    LN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()