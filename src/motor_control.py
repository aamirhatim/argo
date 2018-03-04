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

        self.ar_sub = rospy.Subscriber("/visualization_marker", Marker, self.get_pos)
        self.previous = PointStamped()

    def get_pos(self, data):
        location = PointStamped()
        location.header = data.header
        location.point = data.pose.position
        print location.point

        if location.point.z < 0.5:
            print "reverse"
        elif location.point.z < 0.7:
            print "stop"
        else:
            print "forward"



# class luggo_obj:
#     def __init__(self):
#         print "Connecting to motors..."
#         self.motor_status = 0
#         try:
#             self.robo = Roboclaw("/dev/ttyACM0", 115200)
#             self.robo.Open()
#             self.address = 0x80
#         except:
#             print "Failed to connect to motors! Exiting."
#             self.motor_status = 1
#             return
#
#         print "Motors detected!"
#
#         print "Setting up ROS object..."
#         self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.get_dist)
#         # self.pub = rospy.Publisher("/luggo/encoder_count", Encoder)
#         print "Init complete, let's roll homie."
#
#     def get_dist(self, data):
#         temp = data.markers
#         if len(temp) == 1:
#             tag = temp[0]
#             z = tag.pose.pose.position.z
#         else:
#             z = 0
#
#         dist = int(z/(10**300))
#         print dist
#         self.move_luggo(dist)
#
#     def read_encoder(self):
#         enc1 = self.robo.ReadEncM1(self.address)
#         enc2 = self.robo.ReadEncM2(self.address)
#
#         print enc1
#         print enc2
#
#     def move_luggo(self, distance):
#         max_speed = 127
#         freeze = 0
#
#         read_enc()
#
#         if distance == 0:
#             print "Freeze"
#             self.robo.ForwardM1(self.address, freeze)
#             self.robo.ForwardM2(self.address, freeze)
#         if distance > 45:
#             print "Onward march!"
#             self.robo.ForwardM1(self.address, max_speed/4)
#             self.robo.ForwardM2(self.address, max_speed/4)
#         elif distance < 43 and distance > 0:
#             print "Retreat!"
#             self.robo.BackwardM1(self.address, max_speed/4)
#             self.robo.BackwardM2(self.address, max_speed/4)
#         elif distance >= 43 and distance <= 45:
#             print "Hold laddy!"
#             self.robo.ForwardM1(self.address, freeze)
#             self.robo.ForwardM2(self.address, freeze)

def main():
    ar_tracker = AR_control()
    rospy.init_node("luggo_ar_tracker")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
