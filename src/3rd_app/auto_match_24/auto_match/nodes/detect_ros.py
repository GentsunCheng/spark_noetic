#!/usr/bin/env python3
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow
import os
import sys
import cv2
import yolov5
import numpy as np
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
    pathlib.WindowsPath = pathlib.PosixPath

class SparkDetect:
    class __results__:
        def __init__(self):
            self.name = []
            self.x = []
            self.y = []
            self.size_x = []
            self.size_y = []
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
            rospy.logerr(f"加载模型失败:{e}, 开始下载")
            # 下载模型
            url = "https://github.com/GentsunCheng/spark_noetic/releases/latest/download/auto.pt"
            os.system("wget " + url + " -O " + model_path)
            if not os.path.exists(model_path):
                rospy.logerr("下载模型失败")
                os.remove(model_path)
                return
            rospy.loginfo("下载模型成功")
            self.model = yolov5.load(model_path)

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
        results = self.model(image, augment=True)

        # 存储检测结果的列表
        result = self.__results__()

        # 遍历检测结果
        try:
            for *xyxy, conf, cls in results.xyxy[0]:
                # 计算中心点坐标
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                # 画出矩形框
                label = f'{self.model.model.names[int(cls)]} ({center_x},{center_y})'
                cv2.rectangle(image, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (0, 0, 255), 2)
                cv2.putText(image, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                # 计算大小
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                # 画出中心点
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                # 存储中心点坐标,物体名称,置信度和图像
                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.name.append(self.model.model.names[int(cls)])
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

            result.image = image
        except Exception as _:
            pass

        return result


class Detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("result_image",Image, queue_size=1)
        self.object_pub = rospy.Publisher("/objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1, buff_size=2**24)
        self.items = ["wine", "bear", "clock"]
        self.obj_id = {'wine': 46, 'bear': 88, 'clock': 85}
        self.detector = SparkDetect("/home/spark/auto.pt")

    def decect_cmp(self, _):
        self.cmp = True

    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        objArray.header = data.header
        try:
            results = self.detector.detect(image)
            img_bgr = results.image
            for i in range(len(results.name)):
                if (results.name[i] not in self.items) or results.y[i] < 160 or results.confidence[i] < 0.5:
                    continue
                obj = Detection2D()
                obj.header = data.header
                obj_hypothesis = ObjectHypothesisWithPose()
                obj_hypothesis.id = int(self.obj_id[results.name[i]])
                obj_hypothesis.score = results.confidence[i]
                obj.results.append(obj_hypothesis)
                obj.bbox.size_y = int(results.size_y[i])
                obj.bbox.size_x = int(results.size_x[i])
                obj.bbox.center.x = int(results.x[i])
                obj.bbox.center.y = int(results.y[i])
                objArray.detections.append(obj)
        except:
            img_bgr = image
        self.object_pub.publish(objArray)
        img = cv2.cvtColor(img_bgr, cv2.COLOR_RGB2BGR)
        try:
            image_out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)

if __name__=='__main__':
    rospy.init_node('detector_node')
    obj=Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()