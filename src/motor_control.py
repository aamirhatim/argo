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
        self.argo.reset_controller()
        self.argo.reset_encoders()

        self.ref = int(input("Enter max speed (between 0 and 100): ") * self.argo.max_speed/100)
        print "Max speed set to:",self.ref,"QPPS."
        self.argo.session_max = self.ref
                                                # Initialize:
        self.LFspeed = int(self.ref)            # Left wheel forward speed
        self.LBspeed = int(self.ref*(-1))       # Left wheel backward speed
        self.RFspeed = int(self.ref)            # Right wheel forward speed
        self.RBspeed = int(self.ref*(-1))       # Right wheel backward speed

        self.forward_limit = 0.65
        self.back_limit = 0.5
        self.x_limit = 0.1
        self.max_range = 3.0

        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.follow)

        self.previous = PointStamped()
        self.previous_dir = 'x'
        self.previous_turn = 'x'
        self.last_known = AlvarMarkers()
        self.no_tag_count = 0

    def heartbeat(self):
        state = self.argo.read_encoders()
        if (self.no_tag_count <= 2) and len(self.last_known.markers) == 1:
            if self.last_known.markers[0].id == 0:
                self.speed_calc(self.last_known.markers[0])
            return

        if (self.no_tag_count == 5) and (not self.previous_turn == 'n') and (self.previous_dir == 's'):
            # If tag was lost while turning, turn to the last know direction of the tag
            print "TURNING..."
            self.go_to_direction()
            return

        if self.no_tag_count > 6:
            Lspeed = int(state.speedM2*.7)
            Rspeed = int(state.speedM1*.7)
            if Lspeed**2 < 1000:
                Lspeed = 0
            if Rspeed**2 < 1000:
                Rspeed = 0
            self.argo.force_speed(Lspeed, Rspeed)

        if self.no_tag_count > 20:
            if state.speedM1 <= 20:
                self.argo.reset_controller()

    def go_to_direction(self):
        target = self.previous.point
        effort = 1.0
        theta = get_angle(target)                       # Calculate theta
        arc = (self.argo.distance/2.0)*theta            # Calculate arc length to travel
        num_ticks = arc*self.argo.counts_per_m          # Convert arc length to encoder counts
        T = num_ticks/2.0                               # Calculate time period for acceleration control
        state = self.argo.read_encoders()               # Get current encoder info
        enc_final = state.encoderM1 + num_ticks         # Calculate final encoder position after turn

        while effort <= .05:
            curr_state = self.argo.read_encoders()
            remainder = enc_final - curr_state.encoderM1

            # Compute an effort and use it to determine turn speed
            effort = (.65)*np.sin((np.pi*remainder)/T)
            if self.previous_turn == 'l':
                left = int(-1*effort*self.ref)
                right = int(effort*self.ref)
            elif self.previous_turn == 'r':
                left = int(effort*self.ref)
                right = int(-1*effort*self.ref)

            # Add turn speed to current speed and send command to motors
            Lspeed = curr_state.speedM2 + left
            Rspeed = curr_state.speedM1 + right
            self.argo.move(Lspeed, Rspeed)
            # rospy.sleep(.1)

    def get_line_direction(self, z):
        if z <= self.back_limit:
            return 'b'                     # Move backward
        elif z >= self.forward_limit:
            return 'f'                     # Move forward
        else:
            return 's'                     # Stop

    def get_turn_direction(self, x):
        if x <= -self.x_limit:
            return 'l'                      # Turn left
        elif x >= self.x_limit:
            return 'r'                      # Turn right
        else:
            return 'n'                      # No turn

    def stop_turn_speed(self, x):
        # Turn speed Gompertz equation
        a = 0.65
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

    def move_turn_speed(self, x):
        # print "TURN & MOVE!"
        state = self.argo.read_encoders()                       # Read motor states
        wheel_speed = abs((state.speedM2 + state.speedM1)/2.0)  # Get absolute average of wheel speeds
        T = self.argo.session_max*2                             # Set period equal to the max speed

        # Slow down Argo if x displacement is large so it has a better chance of seeing the tag when turning
        (left, right) = self.stop_turn_speed(x)                 # First get the turn speed as if it were stopped
        left = int(left*.5*np.cos(np.pi*wheel_speed/T))         # Scale down stopped turn speed depending on Argo's current wheel speed
        right = int(right*.5*np.cos(np.pi*wheel_speed/T))
        return (left, right)

    def ramp_down(self):
        state = self.argo.read_encoders()
        left = int(state.speedM2*.7)
        right = int(state.speedM1*.7)
        if left**2 < 1000:
            left = 0
        if right**2 < 1000:
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
        if (not direction == self.previous_dir) or (not turn == self.previous_turn and direction == 's'):
            self.argo.reset_controller()

        # Calculate straight line speed (in QPPS)
        if direction == 's':
            print "stop"
            (Lspeed, Rspeed) = self.ramp_down()

            # Calculate stopped turn speed
            if not turn == 'n':
                (left, right) = self.stop_turn_speed(x_avg)
                Lspeed += left
                Rspeed += right

        elif direction == 'f':
            print "F"
            # Forward speed governing Gompertz equation: y = e^(b*e^(c*s))
            a = 1                               # Equation amplitude
            b = -4.0                            # Shifts equations along x-axis
            c = -2.0                            # Adjusts growth scaling of function
            s = z_avg - self.forward_limit      # Location in space centered at forward_limit
            effort = a*exp(b*exp(c*s))          # Effort is percentage of max speed (self.ref)
            speed = int(effort*self.ref)
            Lspeed = speed
            Rspeed = speed

            # Calculate turn speed
            if not turn == 'n':
                (left, right) = self.move_turn_speed(x_avg)
                # print left, right
                Lspeed += left
                Rspeed += right

        elif direction == 'b':
            # Backward speed governing Gompertz equation
            a = 0.8                             # Max reverse speed will be 80% of max speed
            b = -5
            c = -11
            s = self.back_limit - z_avg         # Location centered at back_limit
            effort = a*exp(b*exp(c*s))
            speed = int(-1*effort*self.ref)     # Multiply by -1 to reverse motor rotation
            Lspeed = speed
            Rspeed = speed

        # Send speeds to PID controller
        print Lspeed, Rspeed
        self.argo.move(Lspeed, Rspeed)

        # Update previous location
        self.previous = location
        self.previous_dir = direction
        self.previous_turn = turn

    def follow(self, data):
        battery = self.argo.check_battery()
        print battery

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
        self.last_known = data
        return

def main():
    rospy.init_node("argo_ar_tracker")
    ar_tracker = AR_control()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main()
