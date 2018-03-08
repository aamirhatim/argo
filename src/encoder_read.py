#!/usr/bin/env python

###########################
## READ ENCODERS & SPEED ##
###########################
# This code simply reads the encoder and motor speed values at 10Hz
# data is published to the "/argo/encoders" topic

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
import rospy
import time

def main():
    argo = Argo()
    if argo.motor_status == 1:
        return 0
    rospy.init_node("argo_encoder_reader")

    while True:
        argo.read_encoders()
        rospy.sleep(.1)

if __name__ == "__main__":
    main()
