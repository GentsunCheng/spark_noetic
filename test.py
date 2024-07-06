#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import os
import cv2
import yolov5

import platform
import pathlib
plt = platform.system()
if plt != 'Windows':
  pathlib.WindowsPath = pathlib.PosixPath

class YoloDetect:
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
            print("加载模型失败:", e)
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
                # 计算尺寸
                size_x = int(xyxy[2] - xyxy[0])
                size_y = int(xyxy[3] - xyxy[1])
                # 画出中心点
                cv2.circle(image, (center_x, center_y), 5, (255, 0, 0), -1)

                # 存储中心点坐标,物体名称,置信度和图像
                result.name.append(self.model.model.names[int(cls)])
                result.size_x.append(size_x)
                result.size_y.append(size_y)
                result.x.append(center_x)
                result.y.append(center_y)
                result.confidence.append(float(conf))

            result.image = image
        except Exception as e:
            print("未检测到物体:", e)

        self.is_detecting = False

        return result
class CamAction:
    def __init__(self):
        self.detector = YoloDetect("/home/spark/auto.pt")

        self.ids = {"bear": 88, "wine": 46, "clock": 85}

    def detect(self, img):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][2:代表第i个物体的y方向上的位置
        '''
        cube_list = []

        try:
            results = self.detector.detect(img)
        except Exception:
            cube_list.clear()
            return cube_list
        
        index = -1
        # 提取
        for name, in results.name:
            index = index + 1
            cube_list[index][0] = self.ids[name]
            cube_list[index][1][1] = results.x[index]
            cube_list[index][1][2] = results.y[index]

        return cube_list
class ArmAction:
    def __init__(self):
        rospy.init_node('arm_action_node', anonymous=True)
        self.cam = CamAction()
        
        # 添加日志输出，检查话题名称
        image_topic = "/camera/color/image_raw"
        rospy.loginfo(f"Subscribing to image topic: {image_topic}")
        self.img_sub = rospy.Subscriber(image_topic, Image, self.img_callback, queue_size=1, buff_size=2**24)
        self.img = None

        # 获取标定文件数据
        try:
            filename = os.environ['HOME'] + "/thefile.txt"
            rospy.loginfo(f"Reading calibration file: {filename}")
            with open(filename, 'r') as f:
                s = f.read()
            arr = s.split()
            self.x_kb = [float(arr[0]), float(arr[1])]
            self.y_kb = [float(arr[2]), float(arr[3])]
        except Exception as e:
            rospy.logerr("Failed to read calibration file: %s", e)
            rospy.signal_shutdown("Calibration file error")
            return

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)

    def img_callback(self, msg):
        rospy.loginfo("Image callback triggered.")
        try:
            img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        if self.img is None:
            rospy.logwarn("No image received yet.")
            return 0

        # 寻找物品
        cube_list = self.cam.detect(self.img)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0

        cube_list = sorted(cube_list, key=lambda x: x[1][1], reverse=False)

        # 获取机械臂目标位置
        x = self.x_kb[0] * cube_list[0][1][1] + self.x_kb[1]
        y = self.y_kb[0] * cube_list[0][1][0] + self.y_kb[1]
        z = -55

        rospy.loginfo(f"找到物品了！它在: {x}, {y}, {z}")

        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(1)

        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(1)

        # 机械臂移动到目标位置
        self.interface.set_pose(x, y, z)
        rospy.sleep(1)

        # 抬起目标方块
        rospy.loginfo(f"我把物品抬起来了")
        self.interface.set_pose(x, y, z + 120)
        rospy.sleep(2)

        self.grasp_status_pub.publish(String("0"))

        return cube_list[0][0]

if __name__ == '__main__':
    try:
        arm_action = ArmAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
