#!/usr/bin/env python

#########################
## INITIALIZATION NODE ##
#########################
# This node attempts to connect to the Roboclaw and set up all necessary publishers,
# subscribers, and other important variables and functions.

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from roboclaw import Roboclaw                                   # Import RoboClaw library for motor controls
import rospy
from geometry_msgs.msg import Vector3, Twist
from argo.msg import Encoder
from math import sqrt, pi
import time

class Argo:
    def __init__(self):
        print "Trying to connect to motors..."
        self.motor_status = 0
        try:                                                        # First step: connect to Roboclaw controller
            self.port = "/dev/ttyACM0"
            self.argo = Roboclaw(self.port, 115200)
            self.argo.Open()
            self.address = 0x80                                     # Roboclaw address
            self.version = self.argo.ReadVersion(self.address)     # Test connection by getting the Roboclaw version
        except:
            print "Unable to connect to Roboclaw port: ", self.port, "\nCheck your port and setup then try again.\nExiting..."
            self.motor_status = 1
            return

        # Follow through with setup if Roboclaw connected successfully
        # print "Roboclaw detected! Version:", self.version[1]
        print "Setting up..."

        # Set up publishers and subscribers
        self.cmd_sub = rospy.Subscriber("/argo/wheel_speeds", Twist, self.get_wheel_speeds)
        self.encoder = rospy.Publisher("/argo/encoders", Encoder, queue_size = 5)

        # Set class variables
        self.radius = 0.0728                                    # Wheel radius (m)
        self.distance = 0.265                                   # Distance between wheels (m)
        self.max_speed = 13000                                  # Max speed (in QPPS)
        self.scaler = 1                                         # Speed reduction factor
        self.rev_counts = 3200                                  # Encoder clicks per rotation
        self.circ = .4574                                       # Wheel circumference (m)
        self.counts_per_m = int(self.rev_counts/self.circ)      # Encoder counts per meter
        self.conv = self.rev_counts/(2*pi)                      # Wheel speed conversion factor
        self.Lref = 0                                           # Left wheel reference speed
        self.Rref = 0                                           # Right wheel reference speed
        self.Lprevious = 0                                      # Previous time step value for left wheel
        self.Rprevious = 0                                      # Previous time step value for right wheel
        self.Kp = .07                                           # Proportional gain
        self.Kd = .19                                           # Derivative gain
        print "Setup complete, let's roll homie ;)\n\n"

    def pd_control(self, Lspeed, Rspeed):
        feedback = self.read_encoders()
        M1 = feedback.speedM1
        M2 = feedback.speedM2

        print Rspeed
        # print M1
        print Lspeed, "\n"
        # print M2

        Rerror = self.Rref - M1
        Lerror = self.Lref - M2

        Rdot = M1 - self.Rprevious
        Ldot = M2 - self.Lprevious

        Ru = self.Kp*Rerror + self.Kd*Rdot
        Lu = self.Kp*Lerror + self.Kd*Ldot

        self.Rprevious = M1
        self.Lprevious = M2

        newRspeed = int(round(Ru + Rspeed))
        newLspeed = int(round(Lu + Lspeed))

        if newRspeed > self.max_speed:
            newRspeed == self.max_speed
        elif newRspeed < -self.max_speed:
            newRspeed = -self.max_speed
        if newLspeed > self.max_speed:
            newLspeed = self.max_speed
        elif newLspeed < -self.max_speed:
            newLspeed = -self.max_speed

        return (int(newLspeed), int(newRspeed))

    def move(self, Lspeed, Rspeed):
        if Lspeed == 0 and Rspeed == 0:
            self.argo.SpeedM1(self.address, 0)
            self.argo.SpeedM2(self.address, 0)
            self.previous = 0
            return

        (Lspeed, Rspeed) = self.pd_control(Lspeed, Rspeed)
        self.argo.SpeedM1(self.address, Rspeed)
        self.argo.SpeedM2(self.address, Lspeed)

        return (Lspeed, Rspeed)

    def get_velocity(self, vx, vy, ax, ay):
        v = sqrt((vx*vx) + (vy*vy))                             # Linear velocity
        # if vx < 0:
        #     v = -v

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

        # Convert to QPPS
        Rspeed = int(round(Ur*self.conv))
        Lspeed = int(round(Ul*self.conv))

        if Rspeed > 0:
            Rspeed = Rspeed + 80
        elif Rspeed < 0:
            Rspeed = Rspeed - 20

        self.move(Lspeed, Rspeed)

    def read_encoders(self):
        mov = Encoder()
        t = rospy.Time.now()

        # Get encoder values from Roboclaw
        enc2 = self.argo.ReadEncM2(self.address)
        enc1 = self.argo.ReadEncM1(self.address)

        # Get motor speeds
        sp1 = self.argo.ReadSpeedM1(self.address)
        sp2 = self.argo.ReadSpeedM2(self.address)

        # Extract encoder ticks and motor speeds, and publish to /argo/encoders topic
        mov.stamp.secs = t.secs
        mov.stamp.nsecs = t.nsecs
        mov.encoderM1 = enc1[1]
        mov.encoderM2 = enc2[1]
        mov.speedM1 = sp1[1]
        mov.speedM2 = sp2[1]

        self.encoder.publish(mov)
        return mov
