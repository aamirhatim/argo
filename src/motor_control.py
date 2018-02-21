#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
import roboclaw as Roboclaw ## import RoboClaw library for motor controls
import rospy
from ar_track_alvar_msgs.msg import * ## import AR tag custom messages (will remove)

class luggo_obj:
    def __init__(self):
        print "Connecting to motors..."
        try:
            self.robo = Roboclaw("/dev/ttyACM0", 115200)
            self.robo.Open()
        except:
            print "Failed to connect to motors! Exiting."
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

        dist = z/(10**300)
        print "Distance: " + dist
        self.move_luggo(dist)

    def move_luggo(distance):
        address = 0x80
        max_speed = 127
        freeze = 0

        if distance > 1300:
            print "Onward march!"
            self.robo.ForwardM1(address, max_speed/2)
            self.robo.ForwardM2(address, max_speed/2)
        elif distance < 1200:
            print "Reatreat!"
            self.robo.BackwardM1(address, max_speed/2)
            self.robo.BackwardM2(address, max_speed/2)
        elif distance >= 1200 and distance <= 1300:
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
