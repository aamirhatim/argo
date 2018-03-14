#!/usr/bin/env python

##########################
## FOLLOWING A SET PATH ##
##########################
# This code lets you test the v and w calculations for following a line,
# figure eight, and circle.
# Enter a 1, 2 or 3 to begin.
# Use Ctrl+C to stop -- The robot will continue to move after quitting so
# be prepared!
# After quitting, run manual_drive.py to stop the robot.

from math import cos, acos, sin, tan, pi, sqrt
import rospy
import time
import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/argo/lib")
from init_argo import Argo
from geometry_msgs.msg import Vector3, Twist
from scipy.interpolate import UnivariateSpline
import matplotlib.pyplot as pl
import numpy as np


def path(argo):
    rospy.init_node("argo_path")
    pub = rospy.Publisher("/argo/wheel_speeds", Twist, queue_size = 10)
    rate = rospy.Rate(20)

    cmd = 'x'
    vels = Twist()

    while not rospy.is_shutdown():
        if cmd != 'q':
            cmd = raw_input("Enter 1 for line, 2 for figure eight, 3 for circle, 4 for spline:  ")
        else:
            return

        time = rospy.get_time()
        if cmd == '1':
            print "LINE"
            (vels.linear.x, vels.angular.z) = line(time, argo)
        elif cmd == '2':
            print "FIGURE EIGHT"
            (vels.linear.x, vels.angular.z) = eight(time, argo)
        elif cmd == '3':
            print "CIRCLE"
            (vels.linear.x, vels.angular.z) = circle(time, argo)
        elif cmd == '4':
            print "SPLINE"
            t = int(raw_input("Enter time to drive (in seconds): "))
            spline(t, argo, pub)
            cmd = 'q'

        pub.publish(vels)
        rate.sleep()

def line(t, argo):
    # x = sin(pi*t/3)
    vx = (pi/4)*cos(pi*t/4)
    return argo.get_velocity(vx, 0, 0, 0)

def eight(t, argo):
    speed = 5
    T = 2*pi/speed
    # x-Position: x = sin(2*t*T)
    vx = 2*T*cos(2*t*T)
    ax = (-4*T*T)*sin(2*t*T)

    # y-Position: y = sin(t*T)
    vy = T*cos(t*T)
    ay = (-T*T)*sin(t*T)

    return argo.get_velocity(vx, vy, ax, ay)

def circle(t, argo):
    # x = rCos(t*T), y = rSin(t*T), where T = 2*pi/P
    r = 1
    P = 3
    T = 2*pi/P

    vx = -r*T*sin(t*T)
    ax = -r*T*T*cos(t*T)

    vy = r*T*cos(t*T)
    ay = -r*T*T*sin(t*T)

    return argo.get_velocity(vx, vy, ax, ay)

def spline(t, argo, pub):
    # x = sin(pi*t), y = 0
    vels = Twist()
    timespace = np.linspace(0, t, 30)

    Xspline = [sin(pi*timespace[j]) for j in range(len(timespace))]
    Yspline = [0 for i in range(len(timespace))]

    sx = UnivariateSpline(timespace, Xspline, k = 4)
    sy = UnivariateSpline(timespace, Yspline, k = 4)

    for q in range(len(timespace)):
        vx = sx(timespace[q], 1)
        ax = sx(timespace[q], 2)

        vy = sy(timespace[q], 1)
        ay = sy(timespace[q], 2)

        (vels.linear.x, vels.angular.z) = argo.get_velocity(vx, vy, ax, ay)
        print vels
        pub.publish(vels)


def main():
    argo = Argo()
    if argo.motor_status == 1:
        return 0
    try:
        path(argo)
    except rospy.ROSInterruptException:
        print "Shutting down\n"

if __name__ == '__main__':
    main()
