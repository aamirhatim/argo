#!/usr/bin/env python

##########################
## FOLLOWING A SET PATH ##
##########################
# This code lets you test the v and w calculations for following a line, figure eight, and circle.
# Enter a 1, 2 or 3 to begin.
# Use Ctrl+C to stop -- The robot will continue to move after quitting so be prepared!
# After quitting, run manual_drive.py to stop the robot

from math import cos, acos, sin, tan, pi, sqrt
import rospy
import time
import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
from geometry_msgs.msg import Point, Vector3, Twist

def path():
    rospy.init_node("luggo_path")
    pub = rospy.Publisher("/luggo/wheel_speeds", Twist, queue_size = 10)
    rate = rospy.Rate(20)

    cmd = raw_input("Enter 1 for line, 2 for figure eight, 3 for circle:  ")
    vels = Twist()

    while not rospy.is_shutdown():
        time = rospy.get_time()
        if cmd == '1':
            print "LINE"
            (vels.linear.x, vels.angular.z) = line(time)
        elif cmd == '2':
            print "FIGURE EIGHT"
            (vels.linear.x, vels.angular.z) = eight(time)
        elif cmd == '3':
            print "CIRCLE"
            (vels.linear.x, vels.angular.z) = circle(time)

        pub.publish(vels)
        rate.sleep()

def line(t):
    Vx = (pi/2)*cos(pi*t/2)
    return (Vx,0)

def eight(t):
    speed = 5
    T = 2*pi/speed
    # x-Position: x = sin(2*t*T)
    vx = 2*T*cos(2*t*T)
    ax = (-4*T*T)*sin(2*t*T)

    # y-Position: y = sin(t*T)
    vy = T*cos(t*T)
    ay = (-T*T)*sin(t*T)

    # Linear velocity:
    vl = sqrt((vx*vx) + (vy*vy))

    # Angular velocity:
    w = (vy*ax-vx*ay)/((vx*vx)+(vy*vy))

    return (vl,w)

def circle(t):
    # x = rCos(t*T), y = rSin(t*T), where T = 2*pi/P
    r = 1
    P = 3
    T = 2*pi/P

    vx = -r*T*sin(t*T)
    ax = -r*T*T*cos(t*T)

    vy = r*T*cos(t*T)
    ay = -r*T*T*sin(t*T)

    # Linear velocity:
    vl = sqrt((vx*vx) + (vy*vy))

    # Angular velocity:
    w = (vy*ax-vx*ay)/((vx*vx)+(vy*vy))

    return (vl,w)

def main():
    luggo = Luggo()
    try:
        path()
    except rospy.ROSInterruptException:
        print "Shutting down\n"

if __name__ == '__main__':
    main()
