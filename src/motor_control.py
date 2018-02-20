#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
import roboclaw as robo ## import RoboClaw library for motor controls
import rospy
from ar_track_alvar_msgs.msg import AlvarMarker, AlvarMarkers ## import AR tag custom messages (will remove)

def tracker_callback(data):
    temp = AlvarMarker()
    temp = data[0]
    z = temp.pose.position.z
    print z

def move_luggo():
    print "moved!"

def move():
    rospy.init_node("luggo_controller")
    # pub = rospy.Publisher("/luggo/encoder_count", Encoder)
    sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, tracker_callback)

def main():
    try:
        move()
    except rospy.ROSInterruptException:
        pass

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()
