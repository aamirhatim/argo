#!/usr/bin/env python

import rospy
from ar_track_alvar_msgs.msg import *

def tracker_callback(data):
    print data

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
