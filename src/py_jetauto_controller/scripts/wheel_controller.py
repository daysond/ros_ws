#!/usr/bin/env python3
# encoding: utf-8

import time
from functools import reduce
import rospy
from geometry_msgs.msg import Twist


class JetAutoController():

    PI = 3.14159265359
    SPEED = 0.25
    
    pose_x = 0
    pose_y = 0
    yaw = 0

    def __init__(self):
        rospy.init_node('jetauto_teleop', anonymous=True)
        self.publisher_ = rospy.Publisher('/jetauto_controller/cmd_vel', Twist,  queue_size=10)

    def getTwist(self, linear_x=0, linear_y=0, angular_z=0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z

        return msg

    def start(self):
        msg = self.getTwist()
        self.run(msg, 1)
    

    def run(self, msg, duration = 0):
        self.publisher_.publish(msg)
        rospy.sleep(duration)
    
    def stop(self):
        msg = self.getTwist(0.0, 0.0, 0.0)
        self.publisher_.publish(msg)
        print("Publishing: Stop.")
    
    def moveSideWay(self, dir):
        print("Moving side way")
        msg = self.getTwist(0, dir * 0.05, 0)
        self.run(msg)
    
    def moveForward(self, dir):
        print("moving forward")
        msg = self.getTwist(dir * 0.05, 0, 0)
        self.run(msg)
    
    def rotate(self, dir):
        print("Rotating")
        msg = self.getTwist(0, 0, dir * 0.2)
        self.run(msg)
        # self.stop()



