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
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
import csv
import time
from threading import Thread



class LoggerNode(Node):
    """
    _
    """
    def __init__(self):
        super().__init__('Logger')

        self.home = expanduser('~')

        #file = open(strftime(home+'/sim_ws/our_test_logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        
        self.odom = self.create_subscription(
            Odometry, 
            '/ego_racecar/odom', 
            self.save_waypoint_call_back, 
            10)
        self.odom

        # Subsriber to read key pressed.
        self.keysub = self.create_subscription(
            String, 
            '/resetpos', 
            self.key_pressed_call_back, 
            10)
        
        self.doLog = False

    def __del__(self):
        # body of destructor
        self.file.close()

    def key_pressed_call_back(self,data):
        if (data.data == 'k'):
            print("v")
            self.doLog = False
            #open file
            self.file = open(self.home +'/sim_ws/src/bsc_anomaly/test_results/longitude-'+ str(gmtime()[1:6]) +'.csv', 'w')
            header = ['x', 'y', 'yaw', 'speed', 'time']
            self.writer = csv.writer(self.file)
            self.writer.writerow(header)

            # sleep
            time.sleep(0.5)
            # log data
            self.doLog = True
            self.simStart = time.time()



    def save_waypoint_call_back(self,data):
        if self.doLog:
            if self.simStart + 10 < time.time():
                self.doLog = False
                print(data.pose.pose.position.x)
                self.file.close()
                return
            
            quaternion = np.array([data.pose.pose.orientation.x, 
                                data.pose.pose.orientation.y, 
                                data.pose.pose.orientation.z, 
                                data.pose.pose.orientation.w])

            euler = euler_from_quaternion(quaternion)
            speed = LA.norm(np.array([data.twist.twist.linear.x, 
                                    data.twist.twist.linear.y, 
                                    data.twist.twist.linear.z]),2)
            #if data.twist.twist.linear.x>0.:
                #print (data.twist.twist.linear.x)
            self.file.write('%f, %f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                            data.pose.pose.position.y,
                                            euler[2],
                                            speed, time.time()))
    


def main(args=None):
    rclpy.init(args=args)
    LN = LoggerNode()

    rclpy.spin(LN)

    LN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    print("Logger node running")
    main()