#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import *

class ParallelAligner:
    def __init__(self):
        rospy.init_node('parallel_aligner')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.angle_tolerance = 0.05  # 角度容忍度（弧度）
        self.kp = 0.5  # 比例控制器的比例常数

        rospy.loginfo("Parallel aligner initialized.")

    def scan_callback(self, data):
        # 获取前方和后方的距离数据
        front_distances = data.ranges[:len(data.ranges)//4]
        back_distances = data.ranges[3*len(data.ranges)//4:]

        # 计算前后两侧的平均距离
        front_avg = self.calculate_average_distance(front_distances)
        back_avg = self.calculate_average_distance(back_distances)

        rospy.loginfo(f"Front average distance: {front_avg}, Back average distance: {back_avg}")

        # 计算距离差异，判断机器人是否平行于场地边界
        distance_diff = front_avg - back_avg
        rospy.loginfo(f"Distance difference: {distance_diff}")

        if abs(distance_diff) > self.angle_tolerance:
            angle_adjust = self.kp * distance_diff
            self.adjust_angle(angle_adjust)
        else:
            self.stop_robot()

    def calculate_average_distance(self, distances):
        valid_distances = [d for d in distances if not math.isinf(d)]
        if valid_distances:
            return sum(valid_distances) / len(valid_distances)
        else:
            return float('inf')

    def adjust_angle(self, angle):
        # 发布控制命令调整机器人角度
        twist = Twist()
        twist.angular.z = -angle  # 负号因为距离差异与角度调整方向相反
        self.cmd_pub.publish(twist)
        rospy.loginfo(f"Adjusting angle: {angle}")

    def stop_robot(self):
        # 停止机器人
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.loginfo("Stopping robot.")

if __name__ == '__main__':
    try:
        aligner = ParallelAligner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ParallelAligner:
    def __init__(self):
        rospy.init_node('parallel_aligner')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.angle_tolerance = 0.05  # 角度容忍度（弧度）
        self.kp = 0.5  # 比例控制器的比例常数

        rospy.loginfo("Parallel aligner initialized.")

    def scan_callback(self, data):
        # 获取前方和后方的距离数据
        front_distances = data.ranges[:len(data.ranges)//4]
        back_distances = data.ranges[3*len(data.ranges)//4:]

        # 计算前后两侧的平均距离
        front_avg = self.calculate_average_distance(front_distances)
        back_avg = self.calculate_average_distance(back_distances)

        rospy.loginfo(f"Front average distance: {front_avg}, Back average distance: {back_avg}")

        # 计算距离差异，判断机器人是否平行于场地边界
        distance_diff = front_avg - back_avg
        rospy.loginfo(f"Distance difference: {distance_diff}")

        if abs(distance_diff) > self.angle_tolerance:
            angle_adjust = self.kp * distance_diff
            self.adjust_angle(angle_adjust)
        else:
            self.stop_robot()

    def calculate_average_distance(self, distances):
        valid_distances = [d for d in distances if not math.isinf(d)]
        if valid_distances:
            return sum(valid_distances) / len(valid_distances)
        else:
            return float('inf')

    def adjust_angle(self, angle):
        # 发布控制命令调整机器人角度
        twist = Twist()
        twist.angular.z = -angle  # 负号因为距离差异与角度调整方向相反
        self.cmd_pub.publish(twist)
        rospy.loginfo(f"Adjusting angle: {angle}")

    def stop_robot(self):
        # 停止机器人
        twist = Twist()
        self.cmd_pub.publish(twist)
        rospy.loginfo("Stopping robot.")

if __name__ == '__main__':
    try:
        aligner = ParallelAligner()
        rospy.sleep(10.0)
        pub = rospy.Publisher('/task_start', Bool, queue_size=1)
        pub.publish(True)
        rospy.sleep(1.0)
        rospy.loginfo("Task started.")
        aligner.scan_sub.unregister()
        aligner.cmd_pub.unregister()
        del aligner
    except rospy.ROSInterruptException:
        pass
