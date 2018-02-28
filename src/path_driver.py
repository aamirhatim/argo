#!/usr/bin/env python

from math import cos, acos, sin, tan, pi, sqrt
import rospy
import time
import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
from geometry_msgs.msg import Point

def path():
    rospy.init_node("luggo_path")
    pub = rospy.Publisher("/luggo/wheel_speeds", Point, queue_size = 10)
    rate = rospy.Rate(20)
    w = 0
    d = .265
    r = .0715

    cmd = raw_input("Enter 's' to start:  ")

    while cmd == 's' and not rospy.is_shutdown():
        print "moving"
        time = rospy.get_time()
        # (v,w) = line(time)
        # (v,w) = eight(time)
        (v,w) = circle(time)
        Ul = (v-w*d)/r
        Ur = (v+w*d)/r
        # print Ul
        vels = Point()
        vels.x = Ul
        vels.y = Ur
        pub.publish(vels)
        rate.sleep()


def line(t):
    Vx = (pi/2)*cos(pi*t/2)
    return (Vx,0)

def eight(t):
    speed = 14
    # x-Position: x = 3*sin((4*t*pi)/speed)
    vx = (12*pi/speed)*cos(4*pi*t/speed)
    ax = (-48*pi*pi/(speed*speed))*sin(4*pi*t/speed)

    # y-Position: y = 3*sin((2*t*pi)/speed)
    vy = (6*pi/speed)*cos(2*pi*t/speed)
    ay = (-12*pi*pi/(speed*speed))*sin(2*pi*t/speed)

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
