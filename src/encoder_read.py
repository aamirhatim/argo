#!/usr/bin/env python

###########################
## READ ENCODERS & SPEED ##
###########################
# This code simply reads the encoder and motor speed values at 10Hz
# data is published to the "/luggo/encoders" topic

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy
import time

def main():
    luggo = Luggo()
    if luggo.motor_status == 1:
        return 0
    rospy.init_node("luggo_encoder_reader")

    while True:
        luggo.read_encoders()
        rospy.sleep(.1)

if __name__ == "__main__":
    main()
