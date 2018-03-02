#!/usr/bin/env python

######################
## KEYSTROKE PICKUP ##
######################
# This code publishes keyboard commands to the /luggo/key_controls topic every
# time a direction is entered

import rospy
import time
from std_msgs.msg import String

def main():
    rospy.init_node("keyboard_cmd")
    pub = rospy.Publisher("/luggo/key_cmd", String, queue_size = 10)
    direction = "x"

    try:
        while direction != "q":
            direction = raw_input("Choose direction (WASD) then press enter.\nEnter 'e' to stop.\nEnter 'q' to quit:  ")
            print "\n"
            pub.publish(direction)
        print "Exiting..."
    except rospy.ROSInterruptException:
        print "Exiting..."

if __name__ == "__main__":
    main()
