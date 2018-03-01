#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy
import time

def ramp_down(self, speed, prev):
    while speed > 2:
        speed = speed - self.delta
        if prev == 'w':
            self.move_forward(speed)
        elif prev == 's':
            self.move_backward(speed)
        elif prev == 'd':
			self.turn_right(speed)
        elif prev == 'a':
            self.turn_left(speed)

        time.sleep(.05)

def ramp_up(self, speed, direction):
    ramp_speed = 0
    while ramp_speed < speed - self.delta:
        ramp_speed = ramp_speed + self.delta
        if direction == 'w':
            self.move_forward(speed)
        elif direction == 's':
            self.move_backward(speed)
        elif direction == 'd':
			self.turn_right(speed)
        elif direction == 'a':
            self.turn_left(speed)

        time.sleep(.05)

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
