#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from swiftpro.msg import *
from uarm import SwiftAPI


def resetarm(_):
    swift = SwiftAPI("/dev/ttyACM0")
    swift.reset(110.0, 0.0, 35.0)


if __name__ == "__main__":
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, resetarm, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
