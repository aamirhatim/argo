#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from roboclaw import Roboclaw ## import RoboClaw library for motor controls
import rospy
import time

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

        print "Setting up ROS objects..."
        print "Init complete, let's roll homie."

        self.max_speed = 127
        self.scaler = 3
        self.delta = 2
        self.keyboard_move()

    def move_forward(self, speed):
		self.robo.ForwardM1(self.address, speed)
		self.robo.ForwardM2(self.address, speed)

    def move_backward(self, speed):
		self.robo.BackwardM1(self.address, speed)
		self.robo.BackwardM2(self.address, speed)

    def turn_right(self, speed):
		self.robo.BackwardM1(self.address, speed)
		self.robo.ForwardM2(self.address, speed)

    def turn_left(self, speed):
		self.robo.ForwardM1(self.address, speed)
		self.robo.BackwardM2(self.address, speed)

    def rest(self):
		self.robo.ForwardM1(self.address, 0)
		self.robo.ForwardM2(self.address, 0)

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

    def keyboard_move(self):
        speed = self.max_speed/self.scaler
        direction = "x"
        previous = "x"
        while direction != "q":
            previous = direction
            direction = raw_input("Choose direction (WASD) then press enter.\nEnter 'e' to stop.\nEnter 'q' to quit:  ")

            print direction
            if direction == 'w':
                if previous == 's':
                    self.ramp_down(speed, previous)
                self.ramp_up(speed, direction)
                self.move_forward(speed)
            elif direction == 's':
                if previous == 'w':
                    self.ramp_down(speed, previous)
                self.ramp_up(speed, direction)
                self.move_backward(speed)
            elif direction == 'a':
                self.turn_left(speed)
            elif direction =='d':
                self.turn_right(speed)
            elif direction == 'e':
                if previous == 'w' or previous == 's':
                    self.ramp_down(speed, previous)
                self.rest()
            elif direction == 'q':
                self.ramp_down(speed, previous)
                self.rest()

        print "Exiting"

def main():
    luggo = luggo_obj()
    if luggo.motor_status == 1:
        return 0
    rospy.init_node("luggo_controller_manual")

if __name__ == "__main__":
    main()
