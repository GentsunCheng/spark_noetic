#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import socket
import rospy
import math
import cmath
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from swiftpro.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from spark_carry_object.msg import *


class GraspObject():
    '''
    监听主控，用于物品抓取功能
    '''

    def __init__(self):
        '''
        初始化
        '''

        global xc, yc, xc_prev, yc_prev, found_count, angle, debug_mod, mod, block_mod, auto_mod
        angle = 90.0
        debug_mod = 0
        auto_mod = 0
        mod = 0
        block_mod = 0
        self.xc = 0
        self.yc = 0
        self.xc_prev = 0
        self.yc_prev = 0
        self.found_count = 0
        self.is_have_object = False
        self.is_found_object = False
        self.object_union = []
        self.last_object_union = []
        # self.sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_cb, queue_size=10)
        # 订阅机械臂抓取指令
        self.sub2 = rospy.Subscriber(
            '/grasp', String, self.grasp_cp, queue_size=10)
        # 发布机械臂位姿
        self.pub1 = rospy.Publisher(
            'position_write_topic', position, queue_size=10)
        # 发布机械臂吸盘
        self.pub2 = rospy.Publisher(
            'pump_topic', status, queue_size=10)
        # 发布第一关节状态
        self.angle1st_pub = rospy.Publisher(
            'angle1st_topic', angle1st, queue_size=10)
        # 发布第二关节状态
        self.angle2nd_pub = rospy.Publisher(
            'angle2nd_topic', angle2nd, queue_size=10)
        # 发布第三关节状态
        self.angle3rd_pub = rospy.Publisher(
            'angle3rd_topic', angle3rd, queue_size=10)
        # 发布第四关节状态
        self.angle4th_pub = rospy.Publisher(
            'angle4th_topic', angle4th, queue_size=10)
        # 发布机械臂状态
        self.grasp_status_pub = rospy.Publisher(
            'grasp_status', String, queue_size=10)
        # 发布TWist消息控制机器人底盘
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pos = position()
        pos.x = 110.0
        pos.y = 0.0
        pos.z = 35.0
        self.pub1.publish(pos)

    def grasp_cp(self, msg):
        global mod, block_mod, angle, auto_mod
        # 抓取物体
        if msg.data == '0' or msg.data == '0a':
            if msg.data == '0a':
                auto_mod = 1
            # 订阅摄像头话题,对图像信息进行处理
            self.sub = rospy.Subscriber(
                "/camera/rgb/image_raw", Image, self.image_cb, queue_size=10)
            self.is_found_object = False
            rate = rospy.Rate(10)
            times = 0
            steps = 0
            while not self.is_found_object:
                rate.sleep()
                times += 1
                # 转圈没有发现可抓取物体,退出抓取
                if steps >= 2:
                    self.sub.unregister()
                    print("stop grasp\n")
                    status = String()
                    status.data = '-1'
                    self.grasp_status_pub.publish(status)
                    return
                # 旋转一定角度扫描是否有可供抓取的物体
                if times >= 30:
                    times = 0
                    steps += 1
                    print("not found\n")
            print("unregisting sub\n")
            self.sub.unregister()
            print("unregisted sub\n")
            # 抓取检测到的物体
            self.grasp()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        # 释放物体
        if msg.data == '1':
            # 放下物体
            self.is_found_object = False
            self.release_object()
            status = String()
            status.data = '1'
            self.grasp_status_pub.publish(status)

        # 关闭气泵
        if msg.data == '58':
            # 放下物体
            self.is_found_object = False
            self.pub2.publish(0)
            block_mod = 0
            rospy.sleep(0.3)
            self.arm_pose()
            status = String()
            status.data = '1'
            self.grasp_status_pub.publish(status)

        # 机械臂位姿调整
        if msg.data == '51' or msg.data == '52' or msg.data == '53' or msg.data == '666' or msg.data == '55':
            self.is_found_object = False
            if msg.data == '51':
                mod = 0
            elif msg.data == '52':
                mod = 1
            elif msg.data == '53':
                mod = 2
            elif msg.data == '55':
                if block_mod:
                    block_mod = 0
                else:
                    block_mod = 1
            elif msg.data == '666':
                block_mod = 0
                mod = 666
            self.arm_pose()
            status = String()
            status.data = '403'
            self.grasp_status_pub.publish(status)

        # 备选方案
        if msg.data == '200':
            self.is_found_object = False
            self.spare_plan()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        # 扫方块
        if msg.data == '114':
            self.is_found_object = False
            self.swap_left()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        if msg.data == '514':
            self.is_found_object = False
            self.swap_right()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        # 扫方块一
        if msg.data == '1141':
            self.is_found_object = False
            self.swap_square_left()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        if msg.data == '5141':
            self.is_found_object = False
            self.swap_square_right()
            status = String()
            status.data = '0'
            self.grasp_status_pub.publish(status)

        # 第四关节左转
        if msg.data == '41':
            if angle < 180.0:
                angle = angle + 3.0
                self.forth_pose()
        # 第四关节右转
        if msg.data == '43':
            if angle > 0.0:
                angle = angle - 3.0
                self.forth_pose()

        # 机械臂归位
        if msg.data == '403':
            self.is_found_object = False
            times = 0
            steps = 0
            angle = 90.0
            self.default_arm()
            self.forth_pose()
            status = String()
            status.data = '403'
            self.grasp_status_pub.publish(status)

        # 机械臂重置
        if msg.data == '404':
            self.is_found_object = False
            block_mod = 0
            times = 0
            steps = 0
            angle = 90.0
            self.reset_pub.publish(1)
            self.forth_pose()
            status = String()
            status.data = '403'
            self.grasp_status_pub.publish(status)

    # 执行抓取
    def grasp(self):
        global mod, auto_mod
        print("start to grasp\n")
        global found_count
        # stop function

        filename = os.environ['HOME'] + "/thefile.txt"
        file_pix = open(filename, 'r')
        s = file_pix.read()
        file_pix.close()
        print(s)
        arr = s.split()
        a1 = arr[0]
        a2 = arr[1]
        a3 = arr[2]
        a4 = arr[3]
        a = [0]*2
        b = [0]*2
        a[0] = float(a1)
        a[1] = float(a2)
        b[0] = float(a3)
        b[1] = float(a4)
        print('k and b value:', a[0], a[1], b[0], b[1])
        r2 = rospy.Rate(10)
        pos = position()
        # 物体所在坐标+标定误差
        pos.x = a[0] * self.yc_prev + a[1]
        pos.y = b[0] * self.xc_prev + b[1]
        pos.z = -25
        print("z = -25\n")
        self.pub1.publish(pos)
        r2.sleep()

        pos.z = -50
        self.pub1.publish(pos)
        print("z = -50\n")

        # 开始吸取物体
        self.pub2.publish(1)
        r2.sleep()

        # 提起物体
        pos.y = 0.0
        if auto_mod == 1:
            pos.x = 140.0
            pos.z = 150.0
        else:
            pos.x = 250.0
            pos.z = 75.0
        self.pub1.publish(pos)
        mod = 1
        auto_mod = 0

    # 使用CV检测物体
    def image_cb(self, data):
        global xc, yc, xc_prev, yc_prev, found_count
        # 使用 opencv 处理
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image1 = CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')

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
        ret, sure_fg = cv2.threshold(
            dist_transform, 0.5 * dist_transform.max(), 255, 0)

        # 形态学操作，找到未知区域
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        sure_fg = np.uint8(sure_fg)
        sure_fg = cv2.morphologyEx(sure_fg, cv2.MORPH_CLOSE, kernel)
        sure_fg = cv2.erode(sure_fg, None, iterations=4)
        sure_fg = cv2.dilate(sure_fg, None, iterations=4)

        # 寻找轮廓
        contours, hier = cv2.findContours(
            sure_fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 判断是否找到轮廓
        if len(contours) == 0:
            self.is_have_object = False
            self.is_found_object = False
            self.found_count = 0
        else:
            self.is_have_object = True
            index = -1
            p_min = 10000
            self.object_union.clear()
            for i, c in enumerate(contours):
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
                    xc = x_mid
                    yc = y_mid
            # 按抓取长度小到大排序
            self.object_union.sort(key=lambda x: x[0])

            if self.found_count >= 30:
                self.found_count = 0
                self.is_found_object = True
                self.xc = xc
                self.yc = yc
            else:
                # 如果物体没有移动
                if index == -1:
                    self.found_count = 0
                    self.is_found_object = False
                elif abs(xc - self.xc_prev) <= 8 and abs(yc - self.yc_prev) <= 8:
                    self.found_count = self.found_count + 1
                else:
                    self.found_count = 0
        self.xc_prev = xc
        self.yc_prev = yc

    # 释放物体
    def release_object(self):
        global block_mod
        rotate = angle4th()
        pos = position()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_y = client_socket.recv(1024).decode()
        message = 'get_pos_z'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        message = 'disconnect'
        client_socket.sendall(message.encode())
        client_socket.close()
        arr_pos_y = float(response_y)
        arr_pos_z = float(response_z)
        rotate.angle4th = 90
        pos.x = 250.0
        pos.y = int(arr_pos_y)
        if block_mod:
            pos.z = int(arr_pos_z)
        else:
            pos.z = int(arr_pos_z) - 25.0
        self.pub1.publish(pos)
        rospy.sleep(0.5)
        self.pub2.publish(0)
        if block_mod:
            block_mod = 0
            pos.z = int(arr_pos_z) + 25.0
        else:
            pos.z = int(arr_pos_z)
        rospy.sleep(0.5)
        self.pub1.publish(pos)
        self.angle4th_pub.publish(rotate)

    # 机械臂位姿调整
    def arm_pose(self):
        global mod, block_mod
        pos = position()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response = client_socket.recv(1024).decode()
        message = 'disconnect'
        client_socket.sendall(message.encode())
        client_socket.close()
        arr_pos = float(response)
        # go forward
        pos.x = 250.0
        pos.y = int(arr_pos)
        if block_mod:
            if mod == 0:
                pos.z = -50.0
            elif mod == 1:
                pos.z = 50.0
            elif mod == 2:
                pos.z = 150.0
        else:
            if mod == 0:
                pos.z = -25.0
            elif mod == 1:
                pos.z = 75.0
            elif mod == 2:
                pos.z = 175.0
        if mod == 666:
            pos.x = 220.0
            pos.z = -130.0
        self.pub1.publish(pos)
        rospy.sleep(0.5)

    # 第四关节调整
    def forth_pose(self):
        global angle
        rotate = angle4th()
        rotate.angle4th = angle
        self.angle4th_pub.publish(rotate)

    # 备选方案
    def spare_plan(self):
        global block_mod
        pos = position()
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_y = client_socket.recv(1024).decode()
        message = 'get_pos_z'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        message = 'disconnect'
        client_socket.sendall(message.encode())
        client_socket.close()
        arr_pos_y = float(response_y)
        arr_pos_z = float(response_z)
        # go forward
        pos.x = 250.0
        pos.y = int(arr_pos_y)
        if block_mod:
            pos.z = int(arr_pos_z)
        else:
            pos.z = int(arr_pos_z) - 25.0
        self.pub1.publish(pos)
        rospy.sleep(0.3)
        self.pub2.publish(1)
        rospy.sleep(0.5)
        # 提起物体
        pos.x = 250.0
        pos.y = int(arr_pos_y)
        pos.z = int(arr_pos_z)
        self.pub1.publish(pos)

    # 扫方块左
    def swap_left(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        arr_pos_z = float(response_z)
        pos = position()
        pos.x = 90.0
        pos.y = 220.0
        pos.z = arr_pos_z
        self.pub1.publish(pos)
        pos.z = -130.0
        rospy.sleep(0.3)
        self.pub1.publish(pos)

    # 扫方块右
    def swap_right(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        arr_pos_z = float(response_z)
        pos = position()
        pos.x = 90.0
        pos.y = - 220.0
        pos.z = arr_pos_z
        self.pub1.publish(pos)
        pos.z = -130.0
        rospy.sleep(0.3)
        self.pub1.publish(pos)

    # 扫方块一左
    def swap_square_left(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        arr_pos_z = float(response_z)
        pos = position()
        pos.x = 90.0
        pos.y = 220.0
        pos.z = arr_pos_z
        self.pub1.publish(pos)
        pos.z = - 25.0
        rospy.sleep(0.3)
        self.pub1.publish(pos)

    # 扫方块一右
    def swap_square_right(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect(('localhost', 8801))
        message = 'get_pos_y'
        client_socket.sendall(message.encode())
        response_z = client_socket.recv(1024).decode()
        arr_pos_z = float(response_z)
        pos = position()
        pos.x = 90.0
        pos.y = - 220.0
        pos.z = arr_pos_z
        self.pub1.publish(pos)
        pos.z = - 25.0
        rospy.sleep(0.3)
        self.pub1.publish(pos)

    # 机械臂恢复默认位姿
    def default_arm(self):
        pos = position()
        r2 = rospy.Rate(1)
        rotate = angle4th()
        pos.x = 110.0
        pos.y = 0.0
        pos.z = 35.0
        rotate.angle4th = 90
        self.pub1.publish(pos)
        self.angle4th_pub.publish(rotate)
        r2.sleep()
        return 'reset completed'


if __name__ == '__main__':
    try:
        rospy.init_node('GraspObject', anonymous=False)
        rospy.loginfo("Init GraspObject main")
        GraspObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End spark GraspObject main")
