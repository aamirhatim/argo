#!/usr/bin/env python

#########################
## INITIALIZATION NODE ##
#########################
# This node attempts to connect to the Roboclaw and set up all necessary publishers,
# subscribers, and other important variables and functions.

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw                                   # Import RoboClaw library for motor controls
import rospy
from geometry_msgs.msg import Vector3, Twist
from luggo.msg import Encoder
from math import sqrt
import time

class Luggo:
    def __init__(self):
        print "Trying to connect to motors..."
        self.motor_status = 0
        try:                                                        # First step: connect to Roboclaw controller
            self.port = "/dev/ttyACM0"
            self.luggo = Roboclaw(self.port, 115200)
            self.luggo.Open()
            self.address = 0x80                                     # Roboclaw address
            # self.version = self.luggo.ReadVersion(self.address)     # Test connection by getting the Roboclaw version
        except:
            print "Unable to connect to Roboclaw port: ", self.port, "\nCheck your port and setup then try again.\nExiting..."
            self.motor_status = 1
            return

        # Follow through with setup if Roboclaw connected successfully
        # print "Roboclaw detected! Version:", self.version[1]
        print "Setting up..."

        # Set up publishers and subscribers
        self.cmd_sub = rospy.Subscriber("/luggo/wheel_speeds", Twist, self.get_wheel_speeds)
        self.encoder = rospy.Publisher("/luggo/encoders", Encoder, queue_size = 5)

        # Set class variables
        self.radius = 0.0715                                    # Wheel radius (m)
        self.distance = 0.265                                   # Distance between wheels (m)
        self.max_speed = 64                                     # Max speed
        self.Lref = 0                                           # Left wheel reference speed
        self.Rref = 0                                           # Right wheel reference speed
        self.scaler = 1                                         # Speed reduction factor
        self.rev_counts = 3200                                  # Encoder clicks per rotation
        self.circ = .4492                                       # Wheel circumference (m)
        self.counts_per_m = int(self.rev_counts/self.circ)      # Encoder counts per meter
        print "Setup complete, let's roll homie ;)\n\n"

    def move(self, Lspeed, Rspeed):
        print Rspeed
        print Lspeed
        Kp = .5
        sensed = self.read_encoders()
        # print sensed

        # Controller
        Rerror = self.Rref - sensed.speedM1
        Lerror = self.Lref - sensed.speedM2
        Ru = Kp*Rerror
        Lu = Kp*Lerror
        Rspeed = int(round(Ru + Rspeed))
        Lspeed = int(round(Lu + Lspeed))

        # self.luggo.ForwardBackwardM1(self.address, Rspeed)
        # self.luggo.ForwardBackwardM2(self.address, Lspeed)
        self.luggo.SpeedM1(self.address, Rspeed)
        self.luggo.SpeedM2(self.address, Lspeed)
        # print Lspeed
        # print Rspeed

        return (Lspeed, Rspeed)

    def get_velocity(self, vx, vy, ax, ay):
        v = sqrt((vx*vx) + (vy*vy))                             # Linear velocity
        w = (vy*ax-vx*ay)/((vx*vx)+(vy*vy))                     # Angular velocity
        return (v, w)

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
        mov = Encoder()
        t = rospy.Time.now()

        # Get encoder values from Roboclaw
        enc2 = self.luggo.ReadEncM2(self.address)
        enc1 = self.luggo.ReadEncM1(self.address)

        # Get motor speeds
        sp1 = self.luggo.ReadSpeedM1(self.address)
        sp2 = self.luggo.ReadSpeedM2(self.address)

        # Extract encoder ticks and motor speeds, and publish to /luggo/encoders topic
        mov.stamp.secs = t.secs
        mov.stamp.nsecs = t.nsecs
        mov.encoderM1 = enc1[1]
        mov.encoderM2 = enc2[1]
        mov.speedM1 = sp1[1]
        mov.speedM2 = sp2[1]

        self.encoder.publish(mov)
        return mov
