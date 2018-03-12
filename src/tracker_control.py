#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
import rospy
from geometry_msgs.msg import Point

class TrackerControl:
    def __init__(self):
        print "Setting up tracker listener..."
        self.speed = 500
        self.argo = Argo()
        self.sub = rospy.Subscriber("/tracker/obj_center", Point, self.turn_control)
        self.midline = 320
        self.threshhold = 40
        print "Setup complete!"


    def turn_control(self, data):
        delta = data.x - self.midline

        if delta > self.threshhold:
            print "turn right"
            self.argo.Lref = self.speed
            self.argo.Rref = -self.speed
            self.argo.move(self.argo.Lref, self.argo.Rref)

        elif delta < -self.threshhold:
            print "turn left"
            self.argo.Lref = -self.speed
            self.argo.Rref = self.speed
            self.argo.move(self.argo.Lref, self.argo.Rref)

def main():
    rospy.init_node("turn_control")
    tc = TrackerControl()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "We're done here."

if __name__ == "__main__":
    main()
