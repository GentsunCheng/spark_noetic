#!/usr/bin/env python3
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow
import rospy
import cv2
import numpy as np
import yolov5
import time#为复位加的
from uarm.wrapper import SwiftAPI#为复位加的
import os
from sensor_msgs.msg import Image
import sys
from cv_bridge import CvBridge, CvBridgeError
from my_custom_msgs.msg import ImageMessage  # 确保import路径正确
import platform
import pathlib

plt = platform.system()
if plt != 'Windows':
  pathlib.WindowsPath = pathlib.PosixPath

class spark_detect:
    def __init__(self, model_path):
        '''
        初始化YOLOv5检测器
        :param model_path: YOLOv5模型文件路径
        '''
        print("yolo5初始化")
        self.model = yolov5.load(model_path)
        print("yolo5初始化成功")
        self.is_detecting = False
        self.object_pub = rospy.Publisher("objects",ImageMessage, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image, self.detect, queue_size=1)
        self.image_pub = rospy.Publisher("debug_image",Image, queue_size=1)
    def detect(self, data):
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
        print("进入图像")
        try:
            image= CvBridge().imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print('error')
        
        while self.is_detecting:
            rospy.sleep(0.5)
        results = self.model(image, augment=True)
        self.is_detecting = True
        # 存储检测结果的列表
        result=ImageMessage()
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
            print("未检测到物体:", e)
        try:
            image=CvBridge().cv2_to_imgmsg(image, encoding="bgr8")
        except CvBridgeError as e:
            print('error')
            
        self.image_pub.publish(image)
        self.is_detecting = False
        self.object_pub.publish(result)
        print("处理图像")
        return result

def main(args):
    rospy.init_node('detector_node')
    obj=spark_detect("/home/spark/spark_noetic/auto.pt")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)

