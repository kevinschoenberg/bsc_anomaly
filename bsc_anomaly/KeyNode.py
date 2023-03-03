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
import sys
import termios
import tty

class KeyNode(Node):
    """
    _
    """
    def __init__(self):
        super().__init__('Logger')

        self.publisherPos = self.create_publisher(
            String, 
            '/resetpos', 
            10)

    def publisheNow(self,keypressed):
            keypresseds = String()
            keypresseds.data = keypressed
            self.publisherPos.publish(keypresseds)

    
def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# checking system
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    KN = KeyNode()
    settings = saveTerminalSettings()
    try:
        while True:
                key = getKey(settings) #Læser bare input i terminal
                if (key in ['k','K']):
                    print("ss", type(key))
                    KN.publisheNow(str(key))
                    print("ssss")
                elif (key == '\x03'): #ctrl+c
                    break
    except Exception as e:
        print(e)

    rclpy.spin(KN)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    KN.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()