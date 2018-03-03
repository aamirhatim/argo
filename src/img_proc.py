#!/usr/bin/env python

######################
## OBJECT DETECTION ##
######################
# This node uses OpenCV to find the object to track.

import sys
sys.path.insert(0, "/home/aamirhatim/catkin_ws/src/luggo/lib")
from init_luggo import Luggo
import rospy
import cv2
import time
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

def nothing(x):
    pass

class Tracker:
    def __init__(self):
        self.luggo = Luggo()
        if self.luggo.motor_status == 1:
            return
        self.ref = 3000
        self.LFspeed = int(self.ref)
        self.LBspeed = int(self.ref*(-1))
        self.RFspeed = int(self.ref)
        self.RBspeed = int(self.ref*(-1))

        print "Setting up object detector..."
        self.bridge = CvBridge()
        self.img_input = rospy.Subscriber("/raspicam_node/image_raw", Image, self.find_obj)
        # self.obj_loc = rospy.Publisher("/luggo/obj_location", Point, queue_size = 5)
        self.prev = 30

        print "Setup complete. Ready or not, here I come ;)"


    def find_obj(self, data):
        # cv2.namedWindow("Camera")

        # cv2.createTrackbar("H Low","Camera", 0, 179, nothing)
        # cv2.createTrackbar("H High","Camera", 0, 179, nothing)
        # cv2.createTrackbar("S Low","Camera", 0, 255, nothing)
        # cv2.createTrackbar("S High","Camera", 0, 255, nothing)
        # cv2.createTrackbar("V Low","Camera", 0, 255, nothing)
        # cv2.createTrackbar("V High","Camera", 0, 255, nothing)
        #
        # hl = cv2.getTrackbarPos('H Low',"Camera")
        # hh = cv2.getTrackbarPos('H High',"Camera")
        # sl = cv2.getTrackbarPos('S Low',"Camera")
        # sh = cv2.getTrackbarPos('S High',"Camera")
        # vl = cv2.getTrackbarPos('V Low',"Camera")
        # vh = cv2.getTrackbarPos('V High',"Camera")


        try:
            raw_img = self.bridge.imgmsg_to_cv2(data, "bgr8")           # Convert ROS Image to CV image type
        except CvBridgeError, e:
            print "CVBridge Error: ", e
            return

        raw_hsv = cv2.cvtColor(raw_img, cv2.COLOR_BGR2HSV)          # Convert to a HSV colorspace
        # red_lower = np.array([hl, sl, vl])
        # red_upper = np.array([hh, sh, vh])
        red_lower = np.array([0, 170, 30])              # Define upper and lower bounds for masking
        red_upper = np.array([17, 255, 150])
        mask = cv2.inRange(raw_hsv, red_lower, red_upper)

        mask = cv2.erode(mask, None, iterations = 5)            # Erode to filter out noise
        mask = cv2.dilate(mask, None, iterations = 7)           # Dilate to amplify remaining shape

        final = cv2.bitwise_and(raw_hsv, raw_hsv, mask= mask)

        # Find contours within the mask and add them to a list
        contour = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(contour) > 0:
            c = max(contour, key = cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 20:
                if radius < 30:
                    print "forward"
                    self.luggo.Lref = self.ref
                    self.luggo.Rref = self.ref
                    (self.LFspeed, self.RFspeed) = self.luggo.move(self.LFspeed, self.RFspeed)
                elif radius > 50:
                    print "backwards"
                    self.luggo.Lref = int(self.ref*(-1))
                    self.luggo.Rref = int(self.ref*(-1))
                    (self.LBspeed, self.RBspeed) = self.luggo.move(self.LBspeed, self.RBspeed)
                else:
                    print "stop"
                    self.luggo.move(0, 0)
                    self.luggo.Lprevious = 0
                    self.luggo.Rprevious = 0
                    self.LFspeed = int(self.ref)    # Reset speeds to prevent error buildup
                    self.LBspeed = int(self.ref*(-1))
                    self.RFspeed = int(self.ref)
                    self.RBspeed = int(self.ref*(-1))

                r = int(radius)
                center = (int(x), int(y))
                print center
                print r

                delta = (radius-self.prev)
                print delta,"\n"

                self.prev = radius


        # print raw_hsv[319][239]
        cv2.imshow("Camera", final)                                   # Show image
        cv2.waitKey(3)

def main():
    obj = Tracker()
    rospy.init_node("obj_detector")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
