#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
from geometry_msgs.msg import Point
from luggo.msg import Encoder
import time

class Luggo:
    def __init__(self):
        print "Connecting to motors..."
        self.motor_status = 0
        try:
            self.luggo = Roboclaw("/dev/ttyACM0", 115200)
            self.luggo.Open()
            self.address = 0x80
        except:
            print "Failed to connect to motors! Exiting."
            self.motor_status = 1
            return

        print "Motors detected!"

        print "Setting up ROS objects..."
        self.cmd_sub = rospy.Subscriber("/luggo/wheel_speeds", Point, self.move)
        self.encoder = rospy.Publisher("/luggo/encoders", Encoder)
        print "Init complete, let's roll homie."

    def move(self, data):
        address = self.address
        left = data.x + 64
        right = data.y + 64
        print round(left)
        print round(right)

        # self.luggo.ForwardBackwardM1(address, int(right))
        # self.luggo.ForwardBackwardM2(address, int(left))
        # enc = Encoder()
        # t = rospy.Time.now()
        # enc.stamp.secs = t.secs
        # enc.stamp.nsecs = t.nsecs
        # enc.encoder1 = 1
        # enc.encoder2 = 2
        # self.encoder.publish(enc)
