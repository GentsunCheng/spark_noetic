#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import rospy
import math
import cmath
import cv2
import yolov5
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from swiftpro.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


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
            rospy.logerr(f"加载模型失败:{e}")
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
            rospy.logwarn("未检测到物体:", e)

        self.is_detecting = False

        return result

class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''
        
        self.angle = 90.0
        self.block_height = 100
        self.auto_mod = 0
        self.step = 1
        self.pump_up_down_mod = False
        self.detector = spark_detect("/home/spark/spark_noetic/vegetable.pt")
        self.xc_prev = 0
        self.yc_prev = 0
        self.is_have_object = False
        self.is_found_object = False
        self.object_union = []
        self.last_object_union = []

        # 订阅机械臂抓取指令
        self.sub = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=10)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=1)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher(
            'pump_topic', status, queue_size=1)
        # 发布第四关节状态
        self.angle4th_pub = rospy.Publisher(
            'angle4th_topic', angle4th, queue_size=1)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=1)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pos = position()
        pos.x, pos.y, pos.z = 110.0, 0.0, 35.0
        self.arr_pos_z = pos.z
        self.pub1.publish(pos)

    def grasp_cp(self, msg):
        # 释放物体
        if msg.data == 'release':
            # 放下物体
            self.pump_up_down_mod = False
            self.is_found_object = False
            self.release_object()

        # 关闭气泵
        elif msg.data == 'close_pump':
            # 放下物体
            self.is_found_object = False
            self.pub2.publish(0)
            self.pump_up_down_mod = False
            rospy.sleep(0.3)
            self.arm_pose()

        elif msg.data == 'pump_up_down':
            self.pump_up_down_mod = not self.pump_up_down_mod
            self.arm_pose()

        # 备选方案
        elif msg.data == 'spare':
            self.spare_plan()

        # 气泵左转
        elif msg.data == 'pump_left':
            if self.angle < 180.0:
                self.angle = self.angle + 3.0
                self.forth_pose()
        # 气泵右转
        elif msg.data == 'pump_right':
            if self.angle > 0.0:
                self.angle = self.angle - 3.0
                self.forth_pose()

        # 机械臂归位
        elif msg.data == 'return':
            self.angle = 90.0
            self.default_arm()
            self.forth_pose()

        # 机械臂重置
        elif msg.data == 'reset':
            self.pump_up_down_mod = False
            self.angle = 90.0
            self.reset_pub.publish(1)
            self.forth_pose()

        else:
            try:
                oprate = json.loads(msg.data)
            except:
                rospy.logwarn("未知指令")
                return
            finally:
                if oprate["cmd"] == "grab":
                    self.auto_mod = 1
                    self.is_found_object = False
                    if oprate["type"] == "blue_square":
                        self.sub1 = rospy.Subscriber(
                            "/camera/rgb/image_raw", Image, self.blue_grab, queue_size=1)
                    elif oprate["type"] == "vegetable":
                        self.sub1 = rospy.Subscriber(
                            "/camera/rgb/image_raw", Image, self.veg_grab, queue_size=1)

                elif oprate["cmd"] == "catch":
                    self.xc_prev, self.yc_prev = oprate["x"], oprate["y"]
                    self.auto_mod = 0
                    self.grasp()

                elif oprate["cmd"] == "step":
                    self.step = oprate["step"]
                    self.arm_pose()

                elif oprate["cmd"] == "sweep":
                    if oprate["dir"] == "left":
                        self.sweep_square_left()
                    elif oprate["dir"] == "right":
                        self.sweep_square_right()
            
                

    # 执行抓取
    def grasp(self):
        rospy.loginfo("start to grasp")
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        arr = s.split()
        a1, a2, a3, a4 = arr[0], arr[1], arr[2], arr[3]
        a, b = [0] * 2, [0] * 2
        a[0], a[1], b[0], b[1] = float(a1), float(a2), float(a3), float(a4)
        rospy.loginfo(f"k and b value: {a[0]} {a[1]} {b[0]} {b[1]}")
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = a[0] * self.yc_prev + a[1]
        pos.y = b[0] * self.xc_prev + b[1]
        pos.z = -25.0
        rospy.loginfo("z = -25")
        self.pub1.publish(pos)
        r2.sleep()

        pos.z = -50.0
        self.pub1.publish(pos)
        rospy.loginfo("z = -50")

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()

        # 提起物体
        pos.y = 0.0
        if self.auto_mod == 1:
            pos.x, pos.z = 140.0, 150.0
        else:
            pos.x, pos.z = 250.0, 75.0
        self.pub1.publish(pos)
        self.step = 2
        self.auto_mod = 0
        self.xc_prev, self.yc_prev = 0, 0

    # 使用CV检测物体
    def blue_grab(self, data):
        if self.sub1 is not None:
            self.sub1.unregister()
            self.sub1 = None
        # 使用 opencv 处理
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # 获取图像尺寸
        height, width, _ = cv_image1.shape

        # 设置蓝色颜色范围
        LowerBlue = np.array([95, 90, 80])
        UpperBlue = np.array([130, 255, 255])

        # 计算上方区域的起始和结束位置
        start_y = 0
        end_y = int(height * 30 / 100)

        # 将上方区域设为黑色（将上方区域设置为黑色，排除上方可能的干扰信息）
        cv_image1[start_y:end_y, :] = (0, 0, 0)

        # 计算圆心坐标和半径
        center_x = width // 2
        center_y = height - 1
        radius = int(np.sqrt((center_x - center_y) ** 2 + (width // 2) ** 2))

        # 创建一个与图像大小相同的黑色图像
        result = np.zeros_like(cv_image1)

        # 在结果图像上绘制白色的圆形区域（进一步排除不感兴趣的区域）
        cv2.circle(result, (center_x, center_y), radius, (255, 255, 255), -1)

        # 将结果图像与原始图像进行按位与运算，将圆外的区域设置为黑色
        result = cv2.bitwise_and(cv_image1, result)

        # 将结果图像转换为HSV颜色空间
        cv_image2 = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)

        # 创建蓝色掩膜
        mask = cv2.inRange(cv_image2, LowerBlue, UpperBlue)

        # 通过按位与运算，将蓝色物体提取出来
        cv_image3 = cv2.bitwise_and(cv_image2, cv_image2, mask=mask)

        # 将提取的蓝色物体转换为灰度图像
        cv_image4 = cv_image3[:, :, 0]

        # 使用高斯模糊平滑图像，并进行阈值化处理
        blurred = cv2.GaussianBlur(cv_image4, (9, 9), 0)
        (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)

        # 进行形态学闭运算，清除噪点
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
        cv_image5 = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        cv_image5 = cv2.erode(cv_image5, None, iterations=4)
        cv_image5 = cv2.dilate(cv_image5, None, iterations=4)

        # 寻找前景区域
        dist_transform = cv2.distanceTransform(cv_image5, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(
            dist_transform, 0.5 * dist_transform.max(), 255, 0)

        # 形态学操作，找到未知区域
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        sure_fg = np.uint8(sure_fg)
        sure_fg = cv2.morphologyEx(sure_fg, cv2.MORPH_CLOSE, kernel)
        sure_fg = cv2.erode(sure_fg, None, iterations=4)
        sure_fg = cv2.dilate(sure_fg, None, iterations=4)

        # 寻找轮廓
        contours, _ = cv2.findContours(
            sure_fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        xc, yc = 0, 0

        # 判断是否找到轮廓
        if len(contours) == 0:
            self.is_have_object = False
        else:
            self.is_have_object = True
            index = -1
            p_min = 10000
            self.object_union.clear()
            for _, c in enumerate(contours):
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # 计算物体中心坐标
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4

                # 在原图上绘制物体中心点
                cv2.circle(cv_image1, (int(x_mid), int(y_mid)),
                           5, (0, 0, 255), 2)

                # 计算物体的宽度和高度
                w = math.sqrt((box[0][0] - box[1][0]) **
                              2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) **
                              2 + (box[0][1] - box[3][1]) ** 2)
                size = w * h

                # 计算物体的极坐标
                p, theta = cmath.polar(complex(x_mid - 320, 480 - y_mid))
                cn = cmath.rect(p, theta)
                cv2.line(cv_image1, (320, 480), (int(320 + cn.real),
                                                 int(480 - cn.imag)), (255, 0, 0), 2)

                # 判断物体是否满足一些条件
                if p > 350:
                    continue

                cv2.circle(cv_image1, (int(x_mid), int(y_mid)),
                           10, (0, 0, 255), 2)

                # 将物体信息添加到列表中
                self.object_union.append((p, theta, w, h, size, x_mid, y_mid))

                # 找到最近的物体
                if p < p_min:
                    index = index + 1
                    p_min = p
                    xc, yc = x_mid, y_mid
            # 按抓取长度小到大排序
            self.object_union.sort(key=lambda x: x[0])

        self.xc_prev, self.yc_prev = xc, yc
        if self.is_found_object:
            self.grasp()
        else:
            rospy.loginfo("No object found.")

    # 抓取蔬菜方块
    def veg_grab(self, data):
        if self.sub1 is not None:
            self.sub1.unregister()
            self.sub1 = None
        # 使用 opencv 处理
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image_bgr  = CvBridge().imgmsg_to_cv2(data, "bgr8")
            cv_image_rgb = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        result = self.detector.detect(cv_image_rgb)

        # 获取图像的宽度和高度
        height, width, _ = cv_image_rgb.shape
        center_x = width // 2  # 图像底边中心的x坐标
        bottom_y = height  # 图像底边的y坐标

        min_distance = float('inf')
        closest_x = None
        closest_y = None

        # 遍历所有检测到的物体，找出距离底边中心最近的物体
        for x, y in zip(result.x, result.y):
            distance = np.sqrt((x - center_x) ** 2 + (y - bottom_y) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_x = x
                closest_y = y

        if closest_x is not None and closest_y is not None:
            self.xc_prev, self.yc_prev = closest_x, closest_y
            self.is_found_object = True
            self.grasp()
        else:
            self.is_found_object = False
            rospy.loginfo("No object found.")

     # 释放物体
    def release_object(self):
        rotate = angle4th()
        pos = position()
        rotate.angle4th = 90
        pos.x, pos.y = 250.0, 0.0
        self.arr_pos_z = -125 + self.step * self.block_height
        pos.z = self.arr_pos_z if self.pump_up_down_mod else self.arr_pos_z - 25.0
        self.pub1.publish(pos)
        rospy.loginfo(f"释放坐标: x:{pos.x}, y:{pos.y}, z:{pos.z}")
        rospy.sleep(0.5)
        self.pub2.publish(0)
        pos.z, self.pump_up_down_mod = (self.arr_pos_z + 25.0, 0) if self.pump_up_down_mod else (self.arr_pos_z, self.pump_up_down_mod)
        rospy.sleep(0.5)
        self.pub1.publish(pos)
        self.angle4th_pub.publish(rotate)

    # 机械臂位姿调整
    def arm_pose(self):
        pos = position()
        # go forward
        pos.x, pos.y = 250.0, 0.0
        self.arr_pos_z = pos.z = (-150.0 + self.step * self.block_height) if self.pump_up_down_mod else (-125.0 + self.step * self.block_height)
        self.pub1.publish(pos)
        rospy.sleep(0.5)

    # 第四关节调整
    def forth_pose(self):
        rotate = angle4th()
        rotate.angle4th = self.angle
        self.angle4th_pub.publish(rotate)

    # 备选方案
    def spare_plan(self):
        pos = position()
        # go forward
        pos.x, pos.y = 250.0, 0.0
        pos.z = self.arr_pos_z if self.pump_up_down_mod else self.arr_pos_z - 25.0
        self.pub1.publish(pos)
        rospy.sleep(0.3)
        self.pub2.publish(1)
        rospy.sleep(0.5)
        # 提起物体
        pos.x, pos.y, pos.z = 250.0, 0.0, self.arr_pos_z
        self.pub1.publish(pos)

    # 扫方块左
    def sweep_square_left(self):
        pos = position()
        pos.x, pos.y, pos.z = 250.0, 0.0, 120.0
        self.pub1.publish(pos)
        pos.x, pos.y, pos.z = 90.0, 220.0, -25.0
        rospy.sleep(0.5)
        self.pub1.publish(pos)

    # 扫方块右
    def sweep_square_right(self):
        pos = position()
        pos.x, pos.y, pos.z = 250.0, 0.0, 120.0
        self.pub1.publish(pos)
        pos.x, pos.y, pos.z = 90.0, -220.0, -25.0
        rospy.sleep(0.5)
        self.pub1.publish(pos)

    # 机械臂恢复默认位姿
    def default_arm(self):
        pos = position()
        r2 = rospy.Rate(1)
        rotate = angle4th()
        pos.x, pos.y, pos.z = 110.0, 0.0, 35.0
        rotate.angle4th = 90
        self.pub1.publish(pos)
        self.angle4th_pub.publish(rotate)
        r2.sleep()
        return 'return completed'


if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")
