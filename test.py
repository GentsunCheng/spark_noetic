import rospy
from sensor_msgs.msg import *


def check_grasp_state(data):
    i = 0
    for distance in data.ranges:
        if distance == 0:
            continue
        if distance <= 0.3:
            i += 1
        print("times", i)
        


rospy.init_node("check_grasp_state")
lidar_sub = rospy.Subscriber(
    "/scan", LaserScan, check_grasp_state, queue_size=1, buff_size=2**24
)
rospy.spin()