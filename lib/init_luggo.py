#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
from geometry_msgs.msg import Point, Vector3, Twist
from luggo.msg import Encoder
import time

class Luggo:
    def __init__(self):
        print "Connecting to motors..."
        self.motor_status = 0
        try:
            self.luggo = Roboclaw("/dev/ttyACM0", 115200)
            self.luggo.Open()
        except:
            print "Failed to connect to motors! Exiting."
            self.motor_status = 1
            return

        print "Motors detected!"
        print "Setting up..."
        # Set up publishers and subscribers
        self.cmd_sub = rospy.Subscriber("/luggo/wheel_speeds", Twist, self.get_wheel_speeds)
        self.encoder = rospy.Publisher("/luggo/encoders", Encoder)

        # Set class variables
        self.address = 0x80             # Roboclaw address
        self.radius = 0.0715            # Wheel radius (m)
        self.distance = 0.265           # Distance between wheels (m)
        self.max_speed = 64             # Max speed
        self.scaler = 1                 # Speed scaler
        print "Setup complete, let's roll homie ;)"

    def move(self, Rspeed, Lspeed):
        self.read_encoders()
        print Rspeed
        print Lspeed

        self.luggo.ForwardBackwardM1(self.address, Rspeed)
        self.luggo.ForwardBackwardM2(self.address, Lspeed)

    def get_wheel_speeds(self, data):
        r = self.radius
        d = self.distance
        v = data.linear.x
        w = data.angular.z

        # Calculate left/right wheel speeds and round to nearest integer value
        Ul = (v-w*d)/r
        Ur = (v+w*d)/r
        Rspeed = int(round(Ur))
        Lspeed = int(round(Ul))

        self.move(Rspeed+64, Lspeed+64)

    def read_encoders(self):
        enc = Encoder()
        t = rospy.Time.now()

        # Get encoder values from Roboclaw
        enc1 = self.luggo.ReadEncM1(self.address)
        enc2 = self.luggo.ReadEncM2(self.address)

        # Extract encoder ticks and publish to /luggo/encoders
        enc.stamp.secs = t.secs
        enc.stamp.nsecs = t.nsecs
        enc.encoder1 = enc1[1]
        enc.encoder2 = enc2[1]
        self.encoder.publish(enc)
