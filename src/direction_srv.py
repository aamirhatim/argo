#!/usr/bin/env python

###########################
## GET DIRECTION SERVICE ##
###########################
# This service is subscribed to the /argo/key_cmd topic which publishes keystrokes.
# When called, this service returns the most recent key command.

import rospy
from std_msgs.msg import String
from argo.srv import *

class Direction:
    def __init__(self):
        self.direction = "x"
        rospy.Subscriber("/argo/key_cmd", String, self.save_cmd)
        self.s = rospy.Service("get_direction", GetDirection, self.get_cmd)
        print "Direction Server Ready"
        rospy.spin()

    def save_cmd(self, data):
        self.direction = data.data

    def get_cmd(self, req):
        return GetDirectionResponse(self.direction)

def main():
    rospy.init_node("get_direction_srv")

    try:
        d = Direction()
    except KeyboardInterrupt:
        print "Exiting..."

if __name__ == '__main__':
    main()
