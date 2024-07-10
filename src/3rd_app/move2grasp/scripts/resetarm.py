#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from swiftpro.msg import *


def resetarm(data):
    pub = rospy.Publisher(
            "swiftpro_status_topic", status, queue_size=1)
    if data.data == "reset":
        pub.publish(status(0))
        rospy.sleep(0.5)
        pub.publish(status(1))
        rospy.sleep(0.5)


if __name__ == "__main__":
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, resetarm, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
