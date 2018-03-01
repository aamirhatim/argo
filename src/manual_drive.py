#!/usr/bin/env python

##################
## MANUAL DRIVE ##
##################
# Manually drive the robot using the WSAD keys.
# To send a command, the Enter key must be pressed. Otherwise the
# robot will continue to folow the previous command.
# You can change the speed of the robot by adjusting luggo.scaler

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy

def manual_control(luggo):
    # Scale the max speed
    luggo.scaler = 4
    speed = (luggo.max_speed/luggo.scaler)

    # Calculate forward and backward speeds
    Fspeed = luggo.max_speed + speed        # Forward speed
    Bspeed = luggo.max_speed - speed        # Backward speed

    direction = "x"
    previous = "x"
    while direction != "q":
        previous = direction
        direction = raw_input("Choose direction (WASD) then press enter.\nEnter 'e' to stop.\nEnter 'q' to quit:  ")

        if direction == 'w':
            luggo.move(Fspeed , Fspeed)
        elif direction == 's':
            luggo.move(Bspeed, Bspeed)
        elif direction == 'a':
            luggo.move(Bspeed, Fspeed)
        elif direction =='d':
            luggo.move(Fspeed, Bspeed)
        elif direction == 'e':
            luggo.move(64, 64)
        elif direction == 'q':
            luggo.move(64, 64)

    print "Exiting"

def main():
    luggo = Luggo()
    if luggo.motor_status == 1:
        return 0
    rospy.init_node("luggo_manual_control")
    manual_control(luggo)

if __name__ == "__main__":
    main()
