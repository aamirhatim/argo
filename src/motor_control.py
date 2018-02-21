#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages (will remove)

class luggo_obj:
    def __init__(self):
        print "Connecting to motors..."
        self.robo = Roboclaw("/dev/ttyACM0", 115200)
        self.robo.Open()
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

    def move_luggo(self, distance):
        address = 0x80
        max_speed = 127
        freeze = 0
        if distance == 0:
            print "Freeze"
            self.robo.ForwardM1(address, freeze)
            self.robo.ForwardM2(address, freeze)
        if distance > 45:
            print "Onward march!"
            self.robo.ForwardM1(address, max_speed/4)
            self.robo.ForwardM2(address, max_speed/4)
        elif distance < 43 and distance > 0:
            print "Retreat!"
            self.robo.BackwardM1(address, max_speed/4)
            self.robo.BackwardM2(address, max_speed/4)
        elif distance >= 43 and distance <= 45:
            print "Hold laddy!"
            self.robo.ForwardM1(address, freeze)
            self.robo.ForwardM2(address, freeze)

def main():
    luggo = luggo_obj()
    rospy.init_node("luggo_controller")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
