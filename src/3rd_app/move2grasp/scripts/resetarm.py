#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String
from swiftpro.msg import *
from uarm import SwiftAPI
import json

class Rester():
    def __init__(self, port="/dev/ttyACM0"):
        self.swift = SwiftAPI(port)
        
    def resetarm(self, data):
        try:
            cmd = json.loads(data)
            self.swift.reset(x=cmd["x"], y=cmd["y"], z=cmd["z"])
        except:
            self.swift.reset(x=110.0, y=0.0, z=35.0)


if __name__ == "__main__":
    rester = Rester()
    try:
        rospy.init_node('armreset', anonymous=False)
        sub = rospy.Subscriber('/armreset', String, rester.resetarm, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("program interrupted before completion")
        pass
