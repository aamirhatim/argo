#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
import time

REV_COUNTS = 3200
CIRC = .4492
COUNT_PER_M = 7123

class luggo_obj:
    def __init__(self):
        print "Connecting to motors..."
        self.motor_status = 0
        try:
            self.robo = Roboclaw("/dev/ttyACM0", 115200)
            self.robo.Open()
            self.address = 0x80
        except:
            print "Failed to connect to motors! Exiting."
            self.motor_status = 1
            return

        print "Motors detected!"

        print "Setting up ROS objects..."
        print "Init complete, let's roll homie."

        self.encoder_send()

    def encoder_send(self):
        speed = 127/3
        encStart = self.robo.ReadEncM2(self.address)
        distance = 7123
        while (abs(encStart[1] - self.robo.ReadEncM2(self.address)[1]) < 7123):
            self.robo.ForwardM1(self.address, speed)
            self.robo.ForwardM2(self.address, speed)
            enc2 = self.robo.ReadEncM2(self.address)[1]
            print enc2
            # time.sleep(.02)
            # self.robo.ForwardM1(self.address, 0)
            # self.robo.ForwardM2(self.address, 0)
            # time.sleep(1)
        self.robo.ForwardM1(self.address, 0)
        self.robo.ForwardM2(self.address, 0)

def main():
    luggo = luggo_obj()
    if luggo.motor_status == 1:
        return 0
    rospy.init_node("luggo_controller_manual")

if __name__ == "__main__":
    main()
