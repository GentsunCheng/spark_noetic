#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from uarm.wrapper import SwiftAPI

# 连接uarm swift pro，指定设备路径
swift = SwiftAPI(port="/dev/ttyACM0")

def resetarm(data):
    if data.data == "reset":
        swift.reset(timeout=3, x=110, y=0, z=35)



if __name__ == "__main__":
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, resetarm, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
