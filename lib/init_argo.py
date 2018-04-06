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
            # self.version = self.argo.ReadVersion(self.address)     # Test connection by getting the Roboclaw version
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
        self.distance = 0.372                                   # Distance between wheels (m)
        self.max_speed = 13000                                  # Global max speed (in QPPS)
        self.session_max = 13000                                # Max speed for current session (in QPPS)
        self.scaler = 1                                         # Speed reduction factor
        self.rev_counts = 3200                                  # Encoder clicks per rotation
        self.circ = .4574                                       # Wheel circumference (m)
        self.counts_per_m = int(self.rev_counts/self.circ)      # Encoder counts per meter
        self.conv = self.rev_counts/(2*pi)                      # Wheel speed conversion factor
        self.Lref = 0                                           # Left wheel reference speed
        self.Rref = 0                                           # Right wheel reference speed
        self.Lprev_err = 0                                      # Previous error value for left wheel
        self.Rprev_err = 0                                      # Previous error value for right wheel
        self.Kp = .004                                           # Proportional gain
        self.Kd = .001                                           # Derivative gain
        self.Ki = .0004
        self.LEint = 0
        self.REint = 0
        print "Setup complete, let's roll homie ;)\n\n"

    def reset_controller(self):
        self.LEint = 0
        self.REint = 0
        self.Lprev_err = 0
        self.Rprev_err = 0

    def pd_control(self, Lspeed, Rspeed):
        # Controller outputs a percent effort (0 - 100) which will be applied to the reference motor speeds
        feedback = self.read_encoders()
        M1 = feedback.speedM1
        M2 = feedback.speedM2
        # print "ACTUAL:",M2,M1
        # print "DESIRED:",Lspeed,Rspeed

        # Calculate current speed error for both motors
        Lerror = Lspeed - M2
        Rerror = Rspeed - M1
        # print "ERROR:",Lerror,Rerror

        # Calculate derivative error
        Ldot = Lerror - self.Lprev_err
        Rdot = Rerror - self.Rprev_err
        # print "D-ERROR:",Ldot,Rdot

        # Calculate integral error
        self.LEint += Lerror
        self.REint += Rerror

        # Compute effort
        Lu = self.Kp*Lerror + self.Kd*Ldot + self.Ki*self.LEint
        Ru = self.Kp*Rerror + self.Kd*Rdot + self.Ki*self.REint

        if Lu > 100.0:
            Lu = 100.0
        elif Lu < -100.0:
            Lu = -100

        if Ru > 100.0:
            Ru = 100.0
        elif Ru < -100.0:
            Ru = -100.0

        # print "EFFORT:",Lu,Ru

        # Set new L and R speeds
        Lspeed = int((Lu/100)*self.session_max)
        Rspeed = int((Ru/100)*self.session_max)
        # print "CONTROLLED:",Lspeed, Rspeed,"\n"

        self.Rprev_err = Rerror
        self.Lprev_err = Lerror

        return (Lspeed, Rspeed)

    def move(self, Lspeed, Rspeed):
        if Lspeed == 0 and Rspeed == 0:
            self.argo.SpeedM1(self.address, 0)
            self.argo.SpeedM2(self.address, 0)
            return

        (Lspeed, Rspeed) = self.pd_control(Lspeed, Rspeed)
        self.argo.SpeedM1(self.address, Rspeed)
        self.argo.SpeedM2(self.address, Lspeed)

    def force_speed(self, Lspeed, Rspeed):
        self.argo.SpeedM1(self.address, Rspeed)
        self.argo.SpeedM2(self.address, Lspeed)

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

    def reset_encoders(self):
        self.argo.ResetEncoders(self.address)

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

        # self.encoder.publish(mov)
        return mov

    def check_battery(self):
        main = self.argo.ReadMainBatteryVoltage(self.address)
        logic = self.argo.ReadLogicBatteryVoltage(self.address)
        print main, logic
