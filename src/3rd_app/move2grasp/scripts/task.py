#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class Task:
    class InitTask:
        def __init__(self):
            self.cmd = Twist()
            self.walk_vel = rospy.get_param('walk_vel', 0.15)
            self.yaw_rate = rospy.get_param('yaw_rate', 0.5)
            self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

            # PID控制器参数
            self.pid = PID(Kp=2.0, Ki=0.0, Kd=0.1)  # 调整PID参数
            self.target_angle = None  # 目标角度，初始为None
            self.current_angle = 0.0  # 需要订阅里程计或IMU来更新
            self.current_position = (0.0, 0.0)  # 当前坐标

            # 订阅里程计数据
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        def odom_callback(self, msg):
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            _, _, yaw = euler_from_quaternion(orientation_list)
            self.current_angle = yaw

            position = msg.pose.pose.position
            self.current_position = (position.x, position.y)
            rospy.loginfo(f"Current yaw: {math.degrees(yaw)} degrees")
            rospy.loginfo(f"Current position: {self.current_position}")

        def step_run(self, distance):
            start_position = self.current_position  # 获取初始位置
            target_distance = distance  # 目标前进距离

            rate = rospy.Rate(10)  # 10Hz
            while True:
                # 计算当前移动的距离
                current_distance = math.sqrt((self.current_position[0] - start_position[0]) ** 2 +
                                             (self.current_position[1] - start_position[1]) ** 2)
                if current_distance >= target_distance:
                    break

                # 前进
                self.cmd.linear.x = self.walk_vel
                self.cmd.angular.z = 0
                self.move_pub.publish(self.cmd)

                rospy.loginfo(f"Start position: {start_position}")
                rospy.loginfo(f"Current position: {self.current_position}")
                rospy.loginfo(f"Target distance: {target_distance}")
                rospy.loginfo(f"Current distance: {current_distance}")

                rate.sleep()

            # 停止移动
            self.cmd.linear.x = 0
            self.move_pub.publish(self.cmd)

        def step_rot(self, angle_degrees):
            initial_angle = self.current_angle  # 获取初始角度
            target_angle = initial_angle + math.radians(angle_degrees)  # 计算目标角度

            # 确保目标角度在[-pi, pi]范围内
            if target_angle > math.pi:
                target_angle -= 2 * math.pi
            elif target_angle < -math.pi:
                target_angle += 2 * math.pi

            rate = rospy.Rate(10)  # 10Hz

            while abs(target_angle - self.current_angle) > 0.01:  # 精度控制在0.01弧度内
                error = target_angle - self.current_angle
                dt = 1.0 / 10  # 10Hz
                control_signal = self.pid.update(error, dt)

                # 将控制信号限制在合理范围内
                control_signal = max(min(control_signal, 1.0), -1.0)

                self.cmd.linear.x = 0
                self.cmd.angular.z = control_signal
                self.move_pub.publish(self.cmd)

                rospy.loginfo(f"Initial angle: {math.degrees(initial_angle)} degrees")
                rospy.loginfo(f"Target angle: {math.degrees(target_angle)} degrees")
                rospy.loginfo(f"Current angle: {math.degrees(self.current_angle)} degrees")
                rospy.loginfo(f"Control signal: {control_signal}")

                rate.sleep()

            # 停止旋转
            self.cmd.angular.z = 0
            self.move_pub.publish(self.cmd)

    class PickTask:
        def __init__(self):
            pass

    class PlaceTask:
        def __init__(self):
            pass

    def __init__(self):
        self.init_task = Task.InitTask()

    def init(self):
        self.init_task.step_run(0.5)
        print("step_one done")
        rospy.sleep(0.5)

        self.init_task.step_rot(-90)
        print("step_two done")
        rospy.sleep(0.5)

        self.init_task.step_run(1.3)
        print("step_three done")
        rospy.sleep(0.5)

        self.init_task.step_rot(-90)
        print("step_four done")
        rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('task')
    task = Task()
    task.init()
