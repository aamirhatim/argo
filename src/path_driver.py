#!/usr/bin/env python

import numpy as np
from math import cos, acos, sin, tan, pi, sqrt
import rospy
import time
import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
from geometry_msgs.msg import Point

# Diff drive kinematic equations qDot = [phiDot, xDot, yDot].T
# r = .0715       # wheel radius
# d = .265        # distance between wheels
# phi = .5
#
# config = np.array([
#     [-r/(2*d), (r/2)*cos(phi), (r/2)*sin(phi)],
#     [r/(2*d), (r/2)*cos(phi), (r/2)*sin(phi)]
#     ]).T
# print qDot
#
# velocity = np.array([
#
# ])

def path():
    rospy.init_node("luggo_path")
    pub = rospy.Publisher("/luggo/wheel_speeds", Point, queue_size = 10)
    rate = rospy.Rate(20)
    w = 0
    d = .265
    r = .0715

    while not rospy.is_shutdown():
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
    #Ax = (-pi*pi/4)*sin(pi*t/2)
    return (Vx,0)

def eight(t):
    speed = 14
    # x-Position: x = 3*np.sin((4*t*np.pi)/speed)
    vx = (12*np.pi/speed)*np.cos(4*np.pi*t/speed)
    ax = (-48*np.pi*np.pi/(speed*speed))*np.sin(4*np.pi*t/speed)

    # y-Position: y = 3*np.sin((2*t*np.pi)/speed)
    vy = (6*np.pi/speed)*np.cos(2*np.pi*t/speed)
    ay = (-12*np.pi*np.pi/(speed*speed))*np.sin(2*np.pi*t/speed)

    # Linear velocity:
    vl = np.sqrt((vx*vx) + (vy*vy))

    # reduce linear velocity by 25% so turtle stays within bounds of the sim
    # vl = vl*(0.75)

    # Angular velocity:
    w = (vy*ax-vx*ay)/((vx*vx)+(vy*vy))

    return (vl,w)

def circle(t):
    # x = rCos(pi*t), y = rSin(pi*t)
    vx = -.5*pi*sin(pi*t)
    ax = -.5*pi*pi*cos(pi*t)

    vy = .5*pi*cos(pi*t)
    ay = -.5*pi*pi*sin(pi*t)

    # Linear velocity:
    vl = np.sqrt((vx*vx) + (vy*vy))

    # Angular velocity:
    w = (vy*ax-vx*ay)/((vx*vx)+(vy*vy))

    return (vl,w)

def main():
    luggo = Luggo()
    try:
        path()
    except rospy.ROSInterruptException:
         pass

if __name__ == '__main__':
    main()
