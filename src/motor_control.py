#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class AR_control:
    def __init__(self):
        print "Setting up AR tracking..."
        self.luggo = Luggo()
        if self.luggo.motor_status == 1:
            return

        self.ref = 3000
        self.LFspeed = int(self.ref)
        self.LBspeed = int(self.ref*(-1))
        self.RFspeed = int(self.ref)
        self.RBspeed = int(self.ref*(-1))
        self.forward_limit = 0.8
        self.back_limit = 0.7

        self.ar_sub = rospy.Subscriber("/visualization_marker", Marker, self.get_pos)
        self.previous = PointStamped()

    def get_pos(self, data):
        location = PointStamped()
        location.header = data.header
        location.point = data.pose.position
        # print location.point

        # Calculate differences from previous point
        dX = location.point.x - self.previous.point.x
        dZ = location.point.z - self.previous.point.z
        dT = location.header.stamp.to_sec() - self.previous.header.stamp.to_sec()

        # Forward/backward speed calculation
        speed = 0
        if location.point.z < self.back_limit:
            dLimit = (self.back_limit - location.point.z)/self.back_limit
            speed = int(self.ref * dLimit)
            # print speed
        elif location.point.z >= self.forward_limit:
            dLimit = (location.point.z - self.forward_limit)/self.forward_limit
            if self.ref * dLimit >= self.ref:
                speed = self.ref
            else:
                speed = int(self.ref * dLimit)

        # Left/right calculation
        tVel = abs(int(location.point.x * speed))
        if tVel >= (0.4*speed):
            tVel= int(0.4*speed)

        if location.point.z < self.back_limit:
            print "reverse"
            if location.point.x < -0.05: # Turning right
                print "R right"
                Lspeed = int(speed + tVel)
                Rspeed = int(speed)
            elif location.point.x > 0.05: # Turning left
                print "R left"
                Lspeed = int(speed)
                Rspeed = int(speed + tVel)
            else:
                print "R S"
                Lspeed = speed
                Rspeed = speed

            self.luggo.Lref = int(Lspeed*(-1))
            self.luggo.Rref = int(Rspeed*(-1))
            # (self.LBspeed, self.RBspeed) =
            self.luggo.move(self.luggo.Lref, self.luggo.Rref)
        elif location.point.z <= self.forward_limit:
            print "stop"
            self.luggo.move(0, 0)
            self.luggo.Lprevious = 0
            self.luggo.Rprevious = 0
            # self.LFspeed = int(self.ref)    # Reset speeds to prevent error buildup
            # self.LBspeed = int(self.ref*(-1))
            # self.RFspeed = int(self.ref)
            # self.RBspeed = int(self.ref*(-1))
        else:
            print "forward"
            if location.point.x < -0.1: # Turning left
                print "F left"
                Lspeed = int(speed)
                Rspeed = int(speed + tVel)
            elif location.point.x > 0.1: # Turning right
                print "F right"
                Lspeed = int(speed + tVel)
                Rspeed = int(speed)
            else:
                print "F S"
                Lspeed = speed
                Rspeed = speed
            self.luggo.Lref = int(Lspeed)
            self.luggo.Rref = int(Rspeed)
            # (self.LFspeed, self.RFspeed) =
            self.luggo.move(self.luggo.Lref, self.luggo.Rref)

        self.previous = location

def main():
    ar_tracker = AR_control()
    rospy.init_node("luggo_ar_tracker")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
