#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class ParallelAligner:
    def __init__(self):
        rospy.init_node('parallel_aligner')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.angle_sub = rospy.Subscriber('/target_angle', Float64, self.angle_callback)

        self.current_angle = 0.0
        self.target_angle = None
        self.angle_tolerance = 0.05  # 角度容忍度（弧度）
        self.kp = 0.5  # 比例控制器的比例常数
        self.adjusting = False

        rospy.loginfo("Parallel aligner initialized.")

    def angle_callback(self, msg):
        self.target_angle = msg.data
        self.adjusting = True
        rospy.loginfo(f"Target angle received: {self.target_angle}")

    def scan_callback(self, data):
        if not self.adjusting or self.target_angle is None:
            return  # 如果没有目标角度或不需要调整，则不进行调整

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

        # 计算当前角度
        self.current_angle = math.atan2(distance_diff, front_avg + back_avg)
        rospy.loginfo(f"Current angle: {self.current_angle}")

        angle_error = self.target_angle - self.current_angle
        if abs(angle_error) > self.angle_tolerance:
            angle_adjust = self.kp * angle_error
            self.adjust_angle(angle_adjust)
        else:
            self.stop_robot()
            self.adjusting = False

    def calculate_average_distance(self, distances):
        valid_distances = [d for d in distances if not math.isinf(d)]
        if valid_distances:
            return sum(valid_distances) / len(valid_distances)
        else:
            return float('inf')

    def adjust_angle(self, angle):
        # 发布控制命令调整机器人角度
        twist = Twist()
        twist.angular.z = angle  # 正负号根据实际情况调整
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
