#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
from argo.msg import Encoder
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import numpy as np
from math import exp, degrees

def get_angle(pos):
    theta = np.arctan(pos.x/pos.z)
    return theta

class AR_control:
    def __init__(self):
        print "Setting up AR tracking..."
        self.argo = Argo()
        if self.argo.motor_status == 1:
            return

        self.ref = int(input("Enter max speed (between 0 and 100): ") * self.argo.max_speed/100)
        print "Max speed set to:",self.ref,"QPPS."
        self.argo.session_max = self.ref
                                                # Initialize:
        self.LFspeed = int(self.ref)            # Left wheel forward speed
        self.LBspeed = int(self.ref*(-1))       # Left wheel backward speed
        self.RFspeed = int(self.ref)            # Right wheel forward speed
        self.RBspeed = int(self.ref*(-1))       # Right wheel backward speed

        self.forward_limit = 0.8
        self.back_limit = 0.5
        self.x_limit = 0.1

        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.follow)

        self.previous = PointStamped()
        self.previous_dir = 's'
        self.previous_turn = 'n'
        self.no_tag_count = 0

    def heartbeat(self):
        if self.no_tag_count > 5:
            state = self.argo.read_encoders()
            Lspeed = int(state.speedM2*.7)
            Rspeed = int(state.speedM1*.7)
            if Lspeed**2 < 2500:
                Lspeed = 0
            if Rspeed**2 < 2500:
                Rspeed = 0
            self.argo.force_speed(Lspeed, Rspeed)

        if self.no_tag_count > 10:
            # Reset controller errors
            self.argo.LEint = 0
            self.argo.REint = 0
            self.argo.Lprev_err = 0
            self.argo.Rprev_err = 0


    def get_line_direction(self, z):
        if z <= self.back_limit:
            return 'b'                                      # Move backward
        elif z >= self.forward_limit:
            return 'f'                                      # Move forward
        else:
            return 's'                                      # Stop

    def get_turn_direction(self, x):
        if x_avg <= -self.x_limit:
            return 'l'
        elif x_avg >= self.x_limit:
            return 'r'
        else:
            return 'n'

    def stop_turn_speed(self, x):
        # Turn speed Gompertz equation
        a = 0.7
        b = -5
        c = -9
        s = abs(x) - self.x_limit
        effort = a*exp(b*exp(c*s))
        if x < 0:
            left = int(-1*effort*self.ref)
            right = int(effort*self.ref)
        elif x > 0:
            left = int(effort*self.ref)
            right = int(-1*effort*self.ref)
        else:
            left = 0
            right = 0
        return (left, right)

    def speed_calc(self, data):
        # Get position information of AR tag
        location = PointStamped()
        location.header = data.header
        location.point = data.pose.pose.position

        # Get average x and z position between current and previoius point to reduce noise
        x_avg = (location.point.x + self.previous.point.x)/2.0
        z_avg = (location.point.z + self.previous.point.z)/2.0

        # Get directions in which Argo should move
        direction = self.get_line_direction(z_avg)
        turn = self.get_turn_direction(x_avg)

        # Reset controller errors if needed
        if (not direction == self.previous_dir) or (not turn == self.previous_turn):
            self.argo.reset_controller()

        # Calculate straight line speed (in QPPS)
        if direction == 's':
            # print "stop"
            speed = 0
            Lspeed = 0
            Rspeed = 0

            if not turn == 'n':
                (left, right) = self.stop_turn_speed(x_avg)
                # print left, right
                Lspeed += left
                Rspeed += right
                self.argo.move(Lspeed, Rspeed)

            # Calculate turning speeds
            # if not -self.x_limit <= x_avg <= self.x_limit:
            #     theta = get_angle(location.point)           # Calculate theta
            #     arc = (self.argo.distance/2.0)*theta/2        # Calculate arc length to travel
            #     num_ticks = abs(arc*self.argo.counts_per_m)      # Convert arc length to encoder counts
            #     # print "TICKS:", num_ticks
            #     start = self.argo.read_encoders()           # Get current encoder info
            #     enc1 = abs(start.encoderM1)
            #     # while abs(enc1 - abs(start.encoderM1)) <= num_ticks:
            #     #     state = self.argo.read_encoders()
            #     #     enc1 = abs(state.encoderM1)
            #     #
            #     #     (left, right) = self.stop_turn_speed(location.point.x)
            #     #     Lspeed = left
            #     #     Rspeed = right
            #     #     self.argo.move(Lspeed, Rspeed)
            #     self.argo.move(0,0)
            # else:
            #     self.argo.move(0,0)


        elif direction == 'f':
            # Forward speed governing Gompertz equation: y = e^(b*e^(c*s))
            a = 1                                       # Equation amplitude
            b = -5.0                                    # Shifts equations along x-axis
            c = -3                                      # Adjusts growth scaling of function
            s = z_avg - self.forward_limit   # Location in space centered at forward_limit
            effort = a*exp(b*exp(c*s))                  # Effort is percentage of max speed (self.ref)
            speed = int(effort*self.ref)
            Lspeed = speed
            Rspeed = speed
        elif direction == 'b':
            # Backward speed governing Gompertz equation
            a = 0.8                                     # Max reverse speed will be 80% of max speed
            b = -5
            c = -11
            s = self.back_limit - z_avg      # Location centered at back_limit
            effort = a*exp(b*exp(c*s))
            speed = int(-1*effort*self.ref)             # Multiply by -1 to reverse motor rotation
            Lspeed = speed
            Rspeed = speed

        # Send speeds to PD controller
        self.argo.move(Lspeed, Rspeed)

        # Update previous location
        self.previous = location
        self.previous_dir = direction
        self.previous_turn = turn

    def follow(self, data):
        # Only move if AR tag id 0 is identified
        if not len(data.markers) == 1:
            self.no_tag_count += 1
            print "WHERE ARE YOU?"
        elif not data.markers[0].id == 0:
            self.no_tag_count += 1
            print "THAT'S NOT YOU"
        else:
            self.no_tag_count = 0
            print "I SEE YOU BUD!"
            self.speed_calc(data.markers[0])

        self.heartbeat()
        return


        # Calculate differences from previous point
        dX = location.point.x - self.previous.point.x
        dZ = location.point.z - self.previous.point.z
        dT = location.header.stamp.to_sec() - self.previous.header.stamp.to_sec()

        tVel = 0

        if location.point.z < self.back_limit:
            # print "reverse"
            if location.point.x < -0.05: # Turning right
                # print "R right"
                Lspeed = int(speed + tVel)
                Rspeed = int(speed)
            elif location.point.x > 0.05: # Turning left
                # print "R left"
                Lspeed = int(speed)
                Rspeed = int(speed + tVel)
            else:
                # print "R S"
                Lspeed = speed
                Rspeed = speed

            self.argo.Lref = int(Lspeed)
            self.argo.Rref = int(Rspeed)
            (self.LBspeed, self.RBspeed) = self.argo.move(self.argo.Lref, self.argo.Rref)
        elif location.point.z <= self.forward_limit:
            print "stop"
            # self.argo.move(0, 0)
            # self.argo.Lprevious = 0
            # self.argo.Rprevious = 0
            # self.LFspeed = int(self.ref)    # Reset speeds to prevent error buildup
            # self.LBspeed = int(self.ref*(-1))
            # self.RFspeed = int(self.ref)
            # self.RBspeed = int(self.ref*(-1))
        else:
            # print "forward"
            if location.point.x < -0.1: # Turning left
                # print "F left"
                Lspeed = int(speed)
                Rspeed = int(speed + tVel)
            elif location.point.x > 0.1: # Turning right
                # print "F right"
                Lspeed = int(speed + tVel)
                Rspeed = int(speed)
            else:
                # print "F S"
                Lspeed = speed
                Rspeed = speed
            self.argo.Lref = int(Lspeed)
            self.argo.Rref = int(Rspeed)
            (self.LFspeed, self.RFspeed) = self.argo.move(self.argo.Lref, self.argo.Rref)

        self.previous = location

    # def turn_speed_calc(self, states, d, x):

def main():
    rospy.init_node("argo_ar_tracker")
    ar_tracker = AR_control()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
