#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages (will remove)

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

        print "Setting up ROS object..."
        self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.get_dist)
        # self.pub = rospy.Publisher("/luggo/encoder_count", Encoder)
        print "Init complete, let's roll homie."

    def get_dist(self, data):
        temp = data.markers
        if len(temp) == 1:
            tag = temp[0]
            z = tag.pose.pose.position.z
        else:
            z = 0

        dist = int(z/(10**300))
        print dist
        self.move_luggo(dist)

    def read_encoder(self):
        enc1 = self.robo.ReadEncM1(self.address)
        enc2 = self.robo.ReadEncM2(self.address)

        print enc1
        print enc2

    def move_luggo(self, distance):
        max_speed = 127
        freeze = 0

        read_enc()

        if distance == 0:
            print "Freeze"
            self.robo.ForwardM1(self.address, freeze)
            self.robo.ForwardM2(self.address, freeze)
        if distance > 45:
            print "Onward march!"
            self.robo.ForwardM1(self.address, max_speed/4)
            self.robo.ForwardM2(self.address, max_speed/4)
        elif distance < 43 and distance > 0:
            print "Retreat!"
            self.robo.BackwardM1(self.address, max_speed/4)
            self.robo.BackwardM2(self.address, max_speed/4)
        elif distance >= 43 and distance <= 45:
            print "Hold laddy!"
            self.robo.ForwardM1(self.address, freeze)
            self.robo.ForwardM2(self.address, freeze)

def main():
    luggo = luggo_obj()
    if luggo.motor_status == 0:
        return 0

    rospy.init_node("luggo_controller")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
