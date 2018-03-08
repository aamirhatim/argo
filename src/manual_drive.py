#!/usr/bin/env python

##################
## MANUAL DRIVE ##
##################
# Manually drive the robot using the WSAD keys.
# To send a command, the Enter key must be pressed. Otherwise the
# robot will continue to folow the previous command.
# You can change the speed of the robot by adjusting argo.scaler

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
import rospy
from std_msgs.msg import String
import time
from argo.srv import *

class Manual(object):
    def __init__(self, ref_speed):
        self.pub = rospy.Publisher("/argo/key_cmd", String, queue_size = 5)
        self.argo = Argo()
        if self.argo.motor_status == 1:
            return
        self.ref = ref_speed

        self.LFspeed = int(self.ref)
        self.LBspeed = int(self.ref*(-1))
        self.RFspeed = int(self.ref)
        self.RBspeed = int(self.ref*(-1))
        self.cmd = "x"
        self.prev_cmd = "x"

    def get_cmd(self):
        while self.cmd != "q":              # Call direction service at 20Hz
            self.cmd = get_direction_client()
            self.send_cmd(self.cmd)
            rospy.sleep(.05)
        self.pub.publish("x")

    def send_cmd(self, cmd):
        if self.prev_cmd != "e":                # Stop the robot briefy when changing directions
            if cmd != self.prev_cmd:
                self.argo.move(0, 0)
                rospy.sleep(.3)
                self.argo.Lprevious = 0
                self.argo.Rprevious = 0
                self.LFspeed = int(self.ref)    # Reset speeds to prevent error buildup
                self.LBspeed = int(self.ref*(-1))
                self.RFspeed = int(self.ref)
                self.RBspeed = int(self.ref*(-1))
        self.prev_cmd = cmd

        if cmd == 'w':      # Move forward
            self.argo.Lref = self.ref
            self.argo.Rref = self.ref
            (self.LFspeed, self.RFspeed) = self.argo.move(self.LFspeed, self.RFspeed)

        elif cmd == 's':    # Move backward
            self.argo.Lref = int(self.ref*(-1))
            self.argo.Rref = int(self.ref*(-1))
            (self.LBspeed, self.RBspeed) = self.argo.move(self.LBspeed, self.RBspeed)

        elif cmd == 'a':    # Rotate left
            self.argo.Lref = int(self.ref*(-1))
            self.argo.Rref = self.ref
            (self.LBspeed, self.RFspeed) = self.argo.move(self.LBspeed, self.RFspeed)

        elif cmd == 'd':    # Rotate right
            self.argo.Lref = self.ref
            self.argo.Rref = int(self.ref*(-1))
            (self.LFspeed, self.RBspeed) = self.argo.move(self.LFspeed, self.RBspeed)

        elif cmd == 'e':    # Stop
            self.argo.move(0, 0)

        elif cmd == 'q':    # Stop and exit
            self.argo.move(0, 0)

        else:               # Stop if unknown command
            self.argo.move(0, 0)

def get_direction_client():
    rospy.wait_for_service("get_direction")
    try:
        direction = rospy.ServiceProxy("get_direction", GetDirection)
        command = direction("request")
        return command.response
    except rospy.ServiceException:
        print "Failed"
        return

def main():
    rospy.init_node("argo_manual_control")
    print "Waiting for get_direction service..."
    rospy.wait_for_service("get_direction")

    print "Starting manual drive mode..."
    m = Manual(4000)

    try:
        m.get_cmd()
    except rospy.ROSInterruptException:
        print "Exiting..."


if __name__ == "__main__":
    main()
