#!/usr/bin/env python

##################
## MANUAL DRIVE ##
##################
# Manually drive the robot using the WSAD keys.
# To send a command, the Enter key must be pressed. Otherwise the
# robot will continue to folow the previous command.
# You can change the speed of the robot by adjusting luggo.scaler

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy
from std_msgs.msg import String
import time
from luggo.srv import *

class Manual(object):
    def __init__(self, ref_speed):
        self.pub = rospy.Publisher("/luggo/key_cmd", String, queue_size = 5)
        self.luggo = Luggo()
        if self.luggo.motor_status == 1:
            return
        self.luggo.Lref = ref_speed
        self.luggo.Rref = ref_speed

        self.LFspeed = ref_speed
        self.LBspeed = -ref_speed
        self.RFspeed = ref_speed
        self.RBspeed = -ref_speed
        self.cmd = "x"

    def get_cmd(self):
        while self.cmd != "q":
            self.cmd = get_direction_client()
            self.send_cmd(self.cmd)
            rospy.sleep(.05)
        self.pub.publish("x")

    def send_cmd(self, cmd):
        if cmd == 'w':
            # print "forward"
            (self.LFspeed, self.RFspeed) = self.luggo.move(self.LFspeed, self.RFspeed)
        elif cmd == 's':
            # print "backward"
            (self.LFspeed, self.RFspeed) = self.luggo.move(self.LFspeed, self.RFspeed)
        elif cmd == 'a':
            # print "left"
            (self.LFspeed, self.RFspeed) = self.luggo.move(self.LFspeed, self.RFspeed)
        elif cmd == 'd':
            # print "right"
            (self.LFspeed, self.RFspeed) = self.luggo.move(self.LFspeed, self.RFspeed)
        elif cmd == 'e':
            # print "stop"
            self.luggo.move(0, 0)
        elif cmd == 'q':
            # print "quit"
            self.luggo.move(0, 0)
        else:
            self.luggo.move(0, 0)
            # print "Unknown command, robot stopped."

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
    rospy.init_node("luggo_manual_control")
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
