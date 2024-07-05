#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sklearn.decomposition import PCA
import time

class ParallelAligner:
    def __init__(self):
        rospy.init_node('parallel_aligner')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.angle_tolerance = 0.05  # 角度容忍度（弧度）
        self.kp = 0.05  # 比例控制器的比例常数
        self.ki = 0.1  # 积分常数
        self.kd = 0.2  # 微分常数

        self.previous_error = 0
        self.integral = 0
        self.previous_time = time.time()

        rospy.loginfo("Parallel aligner initialized.")

    def scan_callback(self, data):
        # 将激光雷达数据转换为二维点云
        points = self.convert_to_points(data)

        if points is not None and len(points) > 1:
            # 使用PCA计算点云的主要方向
            angle = self.calculate_angle_with_pca(points)
            rospy.loginfo(f"Calculated angle: {angle} radians")

            # 计算需要修正的角度，使其平行于最近的90度角
            adjustment = self.calculate_adjustment(angle)
            rospy.loginfo(f"Angle adjustment: {adjustment} radians")

            if abs(adjustment) > self.angle_tolerance:
                self.adjust_angle(adjustment)
            else:
                self.stop_robot()

    def convert_to_points(self, data):
        points = []
        angle_min = data.angle_min
        angle_increment = data.angle_increment

        for i, range_value in enumerate(data.ranges):
            if not math.isinf(range_value):
                angle = angle_min + i * angle_increment
                x = range_value * math.cos(angle)
                y = range_value * math.sin(angle)
                points.append([x, y])

        return np.array(points) if points else None

    def calculate_angle_with_pca(self, points):
        pca = PCA(n_components=2)
        pca.fit(points)
        # PCA主成分方向的角度
        angle = math.atan2(pca.components_[0][1], pca.components_[0][0])
        return angle

    def calculate_adjustment(self, angle):
        # 计算与最近的90度的差异
        nearest_90 = round(angle / (math.pi / 2)) * (math.pi / 2)
        return nearest_90 - angle

    def adjust_angle(self, adjustment):
        current_time = time.time()
        dt = current_time - self.previous_time

        # PID控制
        error = adjustment
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # 发布控制命令调整机器人角度
        twist = Twist()
        twist.angular.z = control
        self.cmd_pub.publish(twist)

        self.previous_error = error
        self.previous_time = current_time

        rospy.loginfo(f"Adjusting angle by: {control} radians")

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
