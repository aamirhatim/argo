#!/usr/bin/env python

# This node calls the service to start video capture on the Raspberry Pi (/raspicam_node/start_capture)

import rospy
from std_srvs.srv import Empty

def call_srv():
    rospy.init_node("start_capture", anonymous = True)

    rospy.wait_for_service("/raspicam_node/start_capture")
    start = rospy.ServiceProxy("/raspicam_node/start_capture", Empty)
    start()
    

def main():
    try:
        call_srv()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
