#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from swiftpro.msg import *


def resetarm(data):
    pub1 = rospy.Publisher(
            "swiftpro_status_topic", status, queue_size=1)
    pub2 = rospy.Publisher(
            'position_write_topic', position, queue_size=1)
    if data.data == "reset":
        pub1.publish(status(0))
        rospy.sleep(0.15)
        pub1.publish(status(1))
        rospy.sleep(0.15)
        pos = position()
        pos.x, pos.y, pos.z = 110.0, 0.0, 35.0
        pub2.publish(pos)


if __name__ == "__main__":
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, resetarm, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
