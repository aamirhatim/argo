#!/usr/bin/env python

import rospy
import tf

def main():
    rospy.init_node("luggo_tf")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "usb_cam",
                         "world")
        rate.sleep()

if __name__ == "__main__":
    main()
