#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from swiftpro.msg import *
from uarm import SwiftAPI
from swiftpro.msg import position

class Rester():
    def __init__(self, port="/dev/ttyACM0"):
        self.swift = SwiftAPI(port)
        
    def resetarm(self, _):
        self.swift.reset(x=110.0, y=0.0, z=35.0)
    
    def resetarm_pro(self, data):
        self.swift.reset(x=data.x, y=data.y, z=data.z)


if __name__ == "__main__":
    rester = Rester()
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, rester.resetarm, queue_size=1)
        sub_pro = rospy.Subscriber('/armreset_pro', position, rester.resetarm_pro, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
