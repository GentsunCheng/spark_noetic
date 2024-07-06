#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import yolov5
import math
import cv2
import json
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from std_msgs.msg import *
from spark_carry_object.msg import *
from tf.transformations import euler_from_quaternion


import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath


class spark_detect:
    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.confidence = []
            self.image = None

    def __init__(self, model_path):
        '''
        初始化YOLOv5检测器
        :param model_path: YOLOv5模型文件路径
        '''
        try:
            self.model = yolov5.load(model_path)
        except Exception as e:
            rospy.loginfo("加载模型失败:", e)
        self.is_detecting = False

    def detect(self, image):
        '''
        检测图像中的物体
        :param image: 输入图像
        :return: 结果类结构
                  result.name: 物体名称列表
                  result.x: 物体中心点x坐标列表
                  result.y: 物体中心点y坐标列表
                  result.confidence: 物体置信度列表
                  result.image: 检测后的图像
        '''
        while self.is_detecting:
            rospy.sleep(0.5)
        results = self.model(image, augment=True)
        self.is_detecting = True

        # 存储检测结果的列表
        result = self.__results__()

        # 遍历检测结果
        try:
            for *xyxy, conf, cls in results.xyxy[0]:
                label = f'{self.model.model.names[int(cls)]} {conf:.2f}'
                # 画出矩形框
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 0, 255), 2)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

                # 计算中心点坐标
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                # 画出中心点
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                # 存储中心点坐标,物体名称,置信度和图像
                result.name.append(self.model.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

            result.image = image
        except Exception as e:
            rospy.loginfo("未检测到物体:", e)

        self.is_detecting = False

        return result


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
    class BaseTask:
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

                rate.sleep()

            # 停止旋转
            self.cmd.angular.z = 0
            self.move_pub.publish(self.cmd)

        def step_pick(self, detector, image):
            result = detector.detect(image)

            if result.name:
                # 按照 result.x 的大小排序，并返回对应的 result.name 列表
                sorted_names = [name for _, name in sorted(zip(result.x, result.name))]
                return sorted_names
            else:
                rospy.loginfo("未检测到物体")
                return []
            


    class PickTask:
        def __init__(self):
            self.pub_grab = rospy.Publisher('/grasp', String, queue_size=10)
            pass

        def pick(self, detector, image):
            result = detector.detect(image)

            if result.name:
                # 获取图像的宽度和高度
                height, width, _ = image.shape
                center_x = width // 2  # 图像底边中心的x坐标
                bottom_y = height  # 图像底边的y坐标

                min_distance = float('inf')
                closest_x = None
                closest_y = None
                closest_index = -1

                # 遍历所有检测到的物体，找出距离底边中心最近的物体
                for i, (x, y) in enumerate(zip(result.x, result.y)):
                    distance = np.sqrt((x - center_x) ** 2 + (y - bottom_y) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        closest_index = i
                        closest_x = x
                        closest_y = y
                
                data = {
                    "cmd": "catch",
                    "x": closest_x,
                    "y": closest_y
                }
                self.pub_grab.publish(json.dumps(data))
                rospy.loginfo(f"catched: {result.name[closest_index]}")
                return result.name[closest_index]
                

    class PlaceTask:
        def __init__(self, obj, base_task):
            self.base_task = base_task
            self.obj = obj
            self.main_pub = rospy.Publisher(
                '/grasp', String, queue_size=10)
            self.pos_pub = rospy.Publisher(
                'position_write_topic', position, queue_size=1)

        def __go2b__(self):
            rospy.loginfo("go to b")
            self.base_task.step_rot(90)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            pos = position()
            pos.x, pos.y, pos.z = 10.0, -160.0, 150.0
            self.pos_pub.publish(pos)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)
            data = {
                    "cmd": "step",
                    "step": 1
                }
            self.main_pub.publish(json.dumps(data))
            rospy.sleep(0.5)
            self.main_pub.publish("release")
            rospy.sleep(0.5)
            self.main_pub.publish("return")
            rospy.sleep(1.0)
            self.base_task.step_rot(180)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)
            self.base_task.step_rot(90)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)

        def __go2c__(self):
            rospy.loginfo("go to c")
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            pos = position()
            pos.x, pos.y, pos.z = 10.0, 160.0, 150.0
            self.pos_pub.publish(pos)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            self.base_task.step_rot(90)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)
            data = {
                    "cmd": "step",
                    "step": 1
                }
            self.main_pub.publish(json.dumps(data))
            rospy.sleep(0.5)
            self.main_pub.publish("release")
            rospy.sleep(0.5)
            self.main_pub.publish("return")
            rospy.sleep(1.0)
            self.base_task.step_rot(180)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)

        def __go2d__(self):
            rospy.loginfo("go to d")
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            pos = position()
            pos.x, pos.y, pos.z = 10.0, -160.0, 150.0
            self.pos_pub.publish(pos)
            rospy.sleep(0.5)
            self.base_task.step_run(0.6)
            rospy.sleep(0.5)
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)
            self.base_task.step_run(0.4)
            rospy.sleep(0.5)
            data = {
                    "cmd": "step",
                    "step": 1
                }
            self.main_pub.publish(json.dumps(data))
            rospy.sleep(0.5)
            self.main_pub.publish("release")
            rospy.sleep(0.5)
            self.main_pub.publish("return")
            rospy.sleep(1.0)
            self.base_task.step_rot(180)
            rospy.sleep(0.5)
            self.base_task.step_run(0.4)
            rospy.sleep(0.5)
            self.base_task.step_rot(90)
            rospy.sleep(0.5)
            self.base_task.step_run(1.2)
            rospy.sleep(0.5)
            self.base_task.step_rot(-90)
            rospy.sleep(0.5)

        def place(self, name):
            index = -1
            for i, obj in enumerate(self.obj):
                if obj == name:
                    index = i
                    
            if index == 0:
                self.__go2b__()
            elif index == 1:
                self.__go2c__()
            elif index == 2:
                self.__go2d__()

    def __init__(self):
        self.base_task = self.BaseTask()
        self.pick_task = self.PickTask()
        self.target = None
        self.obj = []
        self.detector = spark_detect("/home/spark/auto.pt")

    def __pick__(self, data):
        if self.img_sub is not None:
            self.img_sub.unregister()
            self.img_sub = None
        # 使用 opencv 处理
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image_bgr  = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.loginfo('CvBridge Error:{e}')
            return
        self.obj = self.base_task.step_pick(self.detector, cv_image_rgb)
        rospy.loginfo(f"Identify result: {self.obj}")

    def __grab__(self, data):
        if self.img_sub_grab is not None:
            self.img_sub_grab.unregister()
            self.img_sub_grab = None
        # 使用 opencv 处理
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image_bgr  = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.loginfo(f'CvBridge Error:{e}')
            return
        self.target = self.pick_task.pick(self.detector, cv_image_rgb)
        rospy.loginfo(f"Identify result: {self.obj}")

    def init(self):
        self.base_task.step_run(0.3)
        rospy.loginfo("step_one done")
        rospy.sleep(0.5)

        self.base_task.step_rot(-90)
        rospy.loginfo("step_two done")
        rospy.sleep(0.5)

        self.base_task.step_run(1.2)
        rospy.loginfo("step_three done")
        rospy.sleep(0.5)

        self.base_task.step_rot(-90)
        rospy.loginfo("step_four done")
        rospy.sleep(0.5)

        self.img_sub = rospy.Subscriber(
                    "/camera/rgb/image_raw", Image, self.__pick__, queue_size=10)
        rospy.loginfo("step_five done")
        rospy.sleep(0.5)

        self.base_task.step_rot(180)
        rospy.loginfo("step_five done")
        rospy.sleep(0.5)

        self.base_task.step_run(0.45)
        rospy.loginfo("step_six done")
        rospy.sleep(0.5)

        self.place_task = self.PlaceTask(self.obj, self.base_task)

    def pick2place(self):
        self.img_sub_grab = rospy.Subscriber(
                    "/camera/rgb/image_raw", Image, self.__grab__, queue_size=10)
        rospy.loginfo("grab done")
        rospy.sleep(1.5)
        self.place_task.place(self.target)
        

def start(bool):
    if bool:
        rospy.loginfo("start")
    try:
        task = Task()
        task.init()
        while True:
            task.pick2place()
    except Exception as e:
        rospy.logerr(f'Error:{e}')

if __name__ == '__main__':
    rospy.init_node('task')
    rospy.sleep(0.5)
    start_sig = rospy.Subscriber('/task_start', Bool, callback=start)
    rospy.spin()

