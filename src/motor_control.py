#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import numpy as np

class AR_control:
    def __init__(self):
        print "Setting up AR tracking..."
        self.argo = Argo()
        if self.argo.motor_status == 1:
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
        self.t = np.linspace(0, 1, 30)
        print self.t
        self.trail = [None for i in range(30)]
        self.trail[0] = self.previous
        self.trail_count = 1

    def get_pos(self, data):
        location = PointStamped()
        location.header = data.header
        location.point = data.pose.position
        # print location.point

        if self.trail_count < 30:
            self.trail[self.trail_count] = location
            self.trail_count += 1
            dT = location.header.stamp.to_sec() - self.previous.header.stamp.to_sec()
            self.previous = location
            print dT
            return
        else:
            print "DONE"
            return

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

            self.argo.Lref = int(Lspeed*(-1))
            self.argo.Rref = int(Rspeed*(-1))
            (self.LBspeed, self.RBspeed) = self.argo.move(self.argo.Lref, self.argo.Rref)
        elif location.point.z <= self.forward_limit:
            print "stop"
            self.argo.move(0, 0)
            self.argo.Lprevious = 0
            self.argo.Rprevious = 0
            self.LFspeed = int(self.ref)    # Reset speeds to prevent error buildup
            self.LBspeed = int(self.ref*(-1))
            self.RFspeed = int(self.ref)
            self.RBspeed = int(self.ref*(-1))
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
            self.argo.Lref = int(Lspeed)
            self.argo.Rref = int(Rspeed)
            (self.LFspeed, self.RFspeed) = self.argo.move(self.argo.Lref, self.argo.Rref)

        self.previous = location

def main():
    ar_tracker = AR_control()
    rospy.init_node("argo_ar_tracker")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
