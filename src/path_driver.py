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
        v = line(time)
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
    return Vx

def main():
    luggo = Luggo()
    try:
        path()
    except rospy.ROSInterruptException:
         pass

if __name__ == '__main__':
    main()
