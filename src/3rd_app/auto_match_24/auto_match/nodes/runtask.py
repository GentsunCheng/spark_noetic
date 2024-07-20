#!/usr/bin/python3

import os
import threading

import cv2
import rospy
import rospkg
import actionlib
import numpy as np
from std_msgs.msg import *
from actionlib_msgs.msg import *
from sensor_msgs.msg import *
from move_base_msgs.msg import *
import common.msg
import common.srv
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction
import swiftpro.msg
from swiftpro.msg import position
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist

class SwiftProInterface:
    def __init__(self):
        # 创建控制机械臂的topic发布者
        self.arm_position_pub = rospy.Publisher(
            "position_write_topic", swiftpro.msg.position, queue_size=1)   # 机械臂运动位置发布者
        self.arm_pump_pub = rospy.Publisher(
            "pump_topic", swiftpro.msg.status, queue_size=1)               # 机械臂气泵状态发布者
        self.arm_status_pub = rospy.Publisher(
            "swiftpro_status_topic", swiftpro.msg.status, queue_size=1)    # 机械臂开关状态发布者


    def set_pose(self, x, y, z, speed = 1000):
        '''
        发布机械臂运动位置
        '''
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        pos.speed = speed
        # rospy.loginfo(f"set pose {x},{y},{z}")
        self.arm_position_pub.publish(pos)
        

    def set_pump(self, enable:bool):
        '''
        吸取或释放，设定机械臂气泵状态
        '''
        rospy.loginfo(f" 设定机械臂气泵状态为：{enable}")
        if enable:
            self.arm_pump_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_pump_pub.publish(swiftpro.msg.status(0))

    def set_status(self, lock:bool):
        '''
        设定机械臂开关状态
        '''
        rospy.loginfo(f"set arm status {lock}")
        if lock:
            self.arm_status_pub.publish(swiftpro.msg.status(1))
        else:
            self.arm_status_pub.publish(swiftpro.msg.status(0))



class CamAction:
    def __init__(self):
        pass

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][0]:代表第i个物体的x方向上的位置;     cube_list[i][1][1]:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            rospy.logerr("can't get object message")
            cube_list.clear()
            return cube_list
        
        # 提取
        for obj in obj_array.detections:
            obj_dist[obj.results[0].id] = [obj.bbox.center.x, obj.bbox.center.y]

        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        for key, value in obj_dist.items():
            cube_list.append([key, value])

        # 按识别次数排列并选择次数最高的物品
        # cube_list.sort(key=lambda x: x[1][0], reverse=True)
        return cube_list
    
    
    def check_if_grasp(self, x, y, timeout=3, confidence=0.5, scope=30):
        stat = False
        total = int(timeout * 4)
        count = 0
        for _ in range(total):
            cube_list = self.detector()
            for pice in cube_list:
                if x - scope < pice[1][0] < x + scope and y - scope < pice[1][1] < y + scope:
                    count += 1
                    break
            rospy.sleep(0.25)
        if count / total > confidence:
            stat = False
            rospy.loginfo("grasp failed")
        else:
            stat = True
            rospy.loginfo("grasp success")
        return stat


    # ======获取物品对应的收取区位置=======
    def get_recriving_area_location(self):
        '''
        获取物体对应的收取区位置
        @return: items_place_dict{} 物品id与收取区对应关系的字典
        键key为物品id,值value为导航地点名称
        '''
        finding = True
        self.robot = RobotMoveAction()
        while(finding):              
            items_place_dict = {} # 创建一个字典放置对饮关系
            cube_list = self.detector() # 获取识别到的物体信息

            if len(cube_list) < 3:
                if len(cube_list) == 0:
                    rospy.logwarn("objects not found...")
                    self.robot.step_go(0.03)
                    rospy.sleep(2)
                    continue
                rospy.logwarn("finding object...")
                self.robot.move_action_cli.send_goal_and_wait(
                    common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=0.1,
                    move_distance=0.03,
                    ),
                rospy.Duration.from_sec(5)  # 超过5s为超时
                )
                rospy.sleep(2)
            elif len(cube_list) >= 3 :
                rospy.loginfo("finded 3 objects")
                # 根据 cube_list[i][1][0] 的值对 cube_list 进行排序
                sorted_cube_list = sorted(cube_list, key=lambda x: x[1][0], reverse=False)

                # 建立对应关系 sorted_cube_list[0][0]代表识别到的物体ID
                items_place_dict[sorted_cube_list[0][0]] = "Collection_B" # 最左边的物体对应收取区D
                items_place_dict[sorted_cube_list[1][0]] = "Collection_C" # 中间的物体对应收取区C
                items_place_dict[sorted_cube_list[2][0]] = "Collection_D" # 最右边的物体对应收取区B
                
                rospy.loginfo("readying to turn back")
                self.robot.move_action_cli.send_goal_and_wait(
                    common.msg.MoveStraightDistanceGoal(
                    type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                    const_rot_vel=-0.1,
                    move_distance=0.1,
                    ), 
                rospy.Duration.from_sec(5)  # 超过5s为超时
                )
                rospy.sleep(3)

                finding = False
                print(items_place_dict)
            
        return items_place_dict



class ArmAction:
    def __init__(self):

        self.cam = CamAction()

        # 获取第一层标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
            f.close()
        del filename
        arr = s.split()
        del s
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]
        arr.clear()
        del arr

        # 获取第二层标定文件数据
        filename = os.environ['HOME'] + "/thefile2.txt"
        with open(filename, 'r') as f:
            s = f.read()
            f.close()
        del filename
        arr = s.split()
        del s
        self.x2_kb = [float(arr[0]), float(arr[1])]
        self.y2_kb = [float(arr[2]), float(arr[3])]
        arr.clear()
        del arr

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.height, self.width = 480, 640
        self.center_x = self.width // 2  # 图像底边中心的x坐标
        self.bottom_y = self.height  # 图像底边的y坐标

        self.time = {46: 0, 88: 0, 85: 0}
        self.block_height = 100
        self.is_in = False
        self.testing = False
        self.complete = {46: False, 88: False, 85: False}
        self.fix_rotate = FixRotate()
        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)
        self.reset_pub = rospy.Publisher("armreset_pro", position, queue_size=1)
        self.lidar_sub = rospy.Subscriber(
            "/scan", LaserScan, self.check_scan_stat, queue_size=1, buff_size=2**24)
        # self.exclude = np.array(cv2.imread(os.path.join(
        #     rospkg.RosPack().get_path('auto_match'),'config', 'tamp.png'), 0))
        self.exclude = np.array(cv2.imread(os.environ['HOME'] + "/spark_noetic/tmp.png", 0))
 
    
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        x = 0
        y = 0
        z = 0
        cube_list = []
        # 寻找物品
        cube_list_tmp = self.cam.detector()
        if len(cube_list_tmp) == 0:
            return 0
        
        for pice in cube_list_tmp:
            xp = pice[1][0]
            yp = pice[1][1]
            if self.exclude[int(yp), int(xp)] >= 128:
                cube_list.append(pice)

        if len(cube_list) == 0:
            return 0
            

        closest_x = cube_list[0][1][0]
        closest_y = cube_list[0][1][1]
        id = cube_list[0][0]
        for pice in cube_list:
            xp = pice[1][0]
            yp = pice[1][1]
            if yp > closest_y:
                closest_x = xp
                closest_y = yp
                id = pice[0]
                
        rospy.loginfo(f"物品位置在: {closest_x}, {closest_y}")
        # 获取机械臂目标位置
        x = self.x_kb[0] * closest_y + self.x_kb[1]
        y = self.y_kb[0] * closest_x + self.y_kb[1]
        z = -50.0
        rospy.loginfo(f"arm: {x}, {y}, {z}")
        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(0.5)
        self.interface.set_pose(x, y, z)
        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(1.0)
        # 抬起目标方块
        rospy.loginfo(f"把物品抬起来")
        self.interface.set_pose(x, y, z + 120)
        rospy.sleep(1.0)
        rospy.loginfo(f"摆到旁边")
        self.arm_default_pose()
        rospy.sleep(0.75)
        if not self.cam.check_if_grasp(closest_x, closest_y, timeout=1.5, confidence=0.3):
            self.reset_pub.publish(position(10, 150, 160, 0))
            return 1
        self.grasp_status_pub.publish(String("0"))
        if self.time[id] < 3:
            self.time[id] += 1
        return id


    def drop(self, item):
        self.complete[item] = False
        if self.time[item] == 3:
            if self.drop_step_three(item):
                return
        if self.time[item] == 2:
            if self.drop_step_two(item):
                return
        if self.time[item] == 1:
            self.drop_step_one(item)

    def drop_step_one(self, _):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        x = 230
        y = 0
        z = 175
        self.interface.set_pose(200, 110, 175)
        rospy.sleep(0.75)
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.25)
        z = -125 + self.block_height
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.5)
        z = z - 25
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.5)
        # 关闭气泵
        self.interface.set_pump(0)
        rospy.sleep(1.0)
        # 向上移动一点
        z = z + 25
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.5)
        self.interface.set_pose(200, 110, 175, 250)
        rospy.sleep(0.75)
        self.arm_default_pose()
        self.grasp_status_pub.publish(String("0"))
        return True
    
    def drop_step_two(self, item):
        found = False
        count = 0
        while not found:
            cube_list_tmp = self.cam.detector()
            time = 0
            while len(cube_list_tmp) == 0 and time < 10:
                time += 1
                rospy.logwarn("list is empty")
                rospy.sleep(0.5)
                cube_list_tmp = self.cam.detector()

            cube_list = []
            for pice in cube_list_tmp:
                xp = pice[1][0]
                yp = pice[1][1]
                if xp < 75 and 430 < yp < 460:
                    continue
                cube_list.append(pice)

            if len(cube_list) == 0:
                if count > 10:
                    self.time[item] = 1
                    return False
                else:
                    continue

            closest_x = cube_list[0][1][0]
            closest_y = cube_list[0][1][1]
            id = cube_list[0][0]

            for pice in cube_list:
                xp = pice[1][0]
                yp = pice[1][1]
                if yp < closest_y:
                    closest_x = xp
                    closest_y = yp
                    id = pice[0]
            
            if closest_x < 100:
                count += 1
                self.fix_rotate.step_rotate_pro(-0.8)
                cube_list_tmp.clear()
                cube_list.clear()
                rospy.sleep(0.75)
            elif closest_x > 540:
                count += 1
                self.fix_rotate.step_rotate_pro(0.8)
                cube_list_tmp.clear()
                cube_list.clear()
                rospy.sleep(0.75)
            else:
                found = True

        if id == item and 40 <= closest_y:
            x = self.x_kb[0] * closest_y + self.x_kb[1]
            y = self.y_kb[0] * closest_x + self.y_kb[1]
            z = 175
            self.interface.set_pose(200, 110, 175)
            rospy.sleep(0.75)
            self.interface.set_pose(x, y, z)
            rospy.sleep(1.25)
            z = -125 + self.block_height * 2
            self.interface.set_pose(x, y, z)
            rospy.sleep(1.5)
            z = z - 25
            self.interface.set_pose(x, y, z)
            rospy.sleep(0.5)
            # 关闭气泵
            self.interface.set_pump(0)
            rospy.sleep(1.0)
            # 向上移动一点
            z = z + 25
            self.interface.set_pose(x, y, z)
            rospy.sleep(0.5)
            self.interface.set_pose(200, 110, 175, 250)
            rospy.sleep(0.75)
            self.arm_default_pose()
            self.grasp_status_pub.publish(String("0"))
            return True
        else:
            self.time[item] = 1
            return False
        

    def drop_step_three(self, item, fantastic=False):
        if fantastic:
            rospy.logwarn("fantastic mode start!!!")
            rospy.sleep(3)
        self.testing = True
        while self.testing:
            rospy.logwarn("check is in")
            rospy.sleep(0.3)

        if self.is_in:
            found = False
            count = 0
            while not found:
                cube_list_tmp = self.cam.detector()
                time = 0
                while len(cube_list_tmp) == 0 and time < 10:
                    time += 1
                    rospy.logwarn("list is empty")
                    rospy.sleep(0.5)
                    cube_list_tmp = self.cam.detector()

                cube_list = []
                for pice in cube_list_tmp:
                    xp = pice[1][0]
                    yp = pice[1][1]
                    if xp < 75 and 430 < yp < 460:
                        continue
                    cube_list.append(pice)

                if len(cube_list) == 0:
                    if count > 10:
                        self.time[item] = 2
                        return False
                    else:
                        continue
                closest_x = cube_list[0][1][0]
                closest_y = cube_list[0][1][1]
                id = cube_list[0][0]

                for pice in cube_list:
                    xp = pice[1][0]
                    yp = pice[1][1]
                    if yp < closest_y:
                        closest_x = xp
                        closest_y = yp
                        id = pice[0]

                if closest_x < 40:
                    count += 1
                    self.fix_rotate.step_rotate_pro(-0.35)
                    cube_list_tmp.clear()
                    cube_list.clear()
                    rospy.sleep(0.75)
                elif closest_x > 600:
                    count += 1
                    self.fix_rotate.step_rotate_pro(0.35)
                    cube_list_tmp.clear()
                    cube_list.clear()
                    rospy.sleep(0.75)
                else:
                    found = True

            if id == item:
                x = self.x2_kb[0] * closest_y + self.x2_kb[1]
                y = self.y2_kb[0] * closest_x + self.y2_kb[1]
                z = -125 + self.block_height * 3
                self.interface.set_pose(200, 110, 175)
                rospy.sleep(0.75)
                self.interface.set_pose(x, y, z)
                rospy.sleep(1.25)
                z = z - 25
                self.interface.set_pose(x, y, z)
                rospy.sleep(0.5)
                # 关闭气泵
                self.interface.set_pump(0)
                rospy.sleep(1.0)
                # 向上移动一点
                z = z + 25
                self.interface.set_pose(x, y, z)
                rospy.sleep(0.5)
                self.interface.set_pose(200, 110, 175, 250)
                rospy.sleep(0.75)
                self.arm_default_pose()
                self.grasp_status_pub.publish(String("0"))
                self.complete[item] = True
                return True
            else:
                self.time[item] = 2
                return False
        else:
            self.time[item] = 2
            return False
        
    def check_scan_stat(self, data):
        tested = False
        rospy.sleep(3.0)
        for _ in range(10):
            for distance in data.ranges:
                tested_once = False
                if distance == 0.0:
                    continue
                if distance < 0.3:
                    tested_once = True
                    break
            if tested_once:
                tested = True
            rospy.sleep(0.3)
            break

        self.is_in = tested

        self.testing = False


    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(1.0)

    def arm_default_pose(self):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(10, 150, 160, 100)
        rospy.sleep(1.0)


class FixRotate:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def step_rotate_pro(self, rad, time=0.5):
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)
        rospy.sleep(time)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)
    


class RobotMoveAction:
    def __init__(self):
        # 创建控制spark直走的action客户端
        self.move_action_cli = actionlib.SimpleActionClient(
            'move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        # 创建控制spark旋转的action客户端
        self.turn_action_cli = actionlib.SimpleActionClient(
            'turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(
            timeout=rospy.Duration.from_sec(3))

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # 创建获取spark前后距离的service客户端
        rospy.wait_for_service('/get_distance')
        self.distance_srv = rospy.ServiceProxy(
            'get_distance', common.srv.GetFrontBackDistance)

        # 创建导航地点的话题发布者
        self.goto_local_pub = rospy.Publisher(
            "mark_nav", String, queue_size=1)

    def goto_local(self, name):
        '''
        根据目标点名称,发布目标位置到MoveBase服务器,根据返回状态进行判断
        @return: True 为成功到达, False 为失败
        '''

        # 发布目标位置
        self.goto_local_pub.publish("go " + name)

        # 设定1分钟的时间限制，进行阻塞等待
        try:
            ret_status = rospy.wait_for_message(
                'move_base/result', MoveBaseActionResult, rospy.Duration(60)).status.status
        except Exception:
            rospy.logwarn("nav timeout!!!")
            ret_status = GoalStatus.ABORTED

        # 如果一分钟之内没有到达，放弃目标
        if ret_status != GoalStatus.SUCCEEDED:
            rospy.Publisher("move_base/cancel", GoalID, queue_size=1).publish(
                GoalID(stamp=rospy.Time.from_sec(0.0), id=""))
            try:
                rospy.wait_for_message(
                    'move_base/result', MoveBaseActionResult, rospy.Duration(3))
            except Exception:
                rospy.logwarn("move_base result timeout. this is abnormal.")
            rospy.loginfo("==========Timed out achieving goal==========")
            return False
        else:
            rospy.loginfo("==========Goal succeeded==========")
            return True

    def adjust_by_tag(self):
        '''
        利用定位码, 调整 spark 在台前位置, 让其对准台面
        @return: True 为调整成功, False 为调整失败
        '''
        # 调整spark与台面的距离，设定预期距离为 0.25m
        walk_distance = self.distance_srv(None).front_distance - 0.25
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_SCAN,
                const_rot_vel = 0.1,
                move_distance = walk_distance,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_back(self, distance=0.2):
        '''
        后退, 用于抓取或放置后使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=-0.1,
                move_distance=distance,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True
    
    def step_go(self, dis):
        '''
        前进, 用于抓取或放置前使用
        @return: True 为调整成功, False 为调整失败
        '''
        self.move_action_cli.send_goal_and_wait(
            common.msg.MoveStraightDistanceGoal(
                type=common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
                const_rot_vel=0.1,
                move_distance=dis,
            ),
            rospy.Duration.from_sec(5)  # 超过5s为超时
        )
        return True

    def step_rotate(self, rad):
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)

    def step_rotate_pro(self, rad, time=0.5):
        twist = Twist()
        twist.angular.z = rad
        self.cmd_pub.publish(twist)
        rospy.sleep(time)
        twist.angular.z = 0
        self.cmd_pub.publish(twist)

    def step_go_pro(self, linear, time=0.3):
        twist_go = Twist()
        twist_go.linear.x = linear
        self.cmd_pub.publish(twist_go)
        rospy.sleep(time)
        twist_go.linear.x = 0
        self.cmd_pub.publish(twist_go)


class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        rospy.loginfo("========ready to task=====")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        rospy.loginfo("========实例化Cam=====")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        rospy.loginfo("========实例化Arm=====")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        rospy.loginfo("========实例化Robot=====")

        # 订阅任务控制指令的话题
        self.task_cmd_sub = rospy.Subscriber("/task_start_flag", String, self.task_cmd_cb) # 订阅任务开始与否信号
        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象
        self.stop_flag = False  # 任务的启停标志

        # 订阅机械臂手动控制的话题
        self.grasp_sub = rospy.Subscriber("grasp", String, self.grasp_cb)

        rospy.loginfo("spark_auto_match_node is ready")

    # 接收到启动自动赛信号，开始执行任务
    def run_task(self):
        ret = False # 是否导航成功标志
        item_type = 0 
        self.arm.arm_home()  # 移动机械臂到其他地方

        # ===== 现在开始执行任务 =====
        rospy.loginfo("start task now.")

        # ==== 离开起始区,避免在膨胀区域中，导致导航失败 =====
        self.robot.step_go(0.3)
        # self.robot.step_rotate(-0.5)

        if self.stop_flag: return

        # ==== 移动机械臂 =====
        self.arm.arm_default_pose()  # 移动机械臂到其他地方

        

        # ===== 导航到分类区 =====
        if self.robot.goto_local("Classification_area"):
            rospy.sleep(2)
            items_place_dict = self.cam.get_recriving_area_location()
        else :
            rospy.logerr("Navigation to Classification_area failed,please run task again ")
            self.stop_flag = True

        sorting_name = "Sorting_DA"

        # =======开始循环运行前往中心区域抓取与放置任务======
        while True:
            # 根据任务安排逐步执行
            print("readying to sort_areas")

            # =====导航到目标地点====

            ret = self.robot.goto_local(sorting_name) # 导航到目标点
            rospy.sleep(0.5) # 停稳

            go_num = 0
            while True:     # 新增##############################################################
                cube_list = self.cam.detector() # 获取识别到的物体信息
                if len(cube_list) < 1:
                    self.robot.step_go_pro(0.2, time=0.5)
                    go_num += 1
                    print("go_num:", go_num)
                    if go_num > 100:
                        self.robot.step_go_pro(0)
                        break
                else:
                    self.robot.step_go_pro(0)
                    break


            if self.stop_flag: return

            # =====识别并抓取物体====
            item_type = 0

            self.arm.reset_pub.publish(position(10, 150, 160, 0))
            if ret: # 判断是否成功到达目标点
                rospy.loginfo("========往前走看清一点=====")
                self.robot.step_go_pro(0.02)  # 前进
                rospy.sleep(1.5) # 停稳
                rospy.loginfo("========扫描中，准备抓取=====")
                item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型
                for i in range(3):
                    if item_type == 1:
                        self.arm.reset_pub.publish(position(10, 150, 160, 0))
                        self.robot.step_go_pro(0.15)
                        rospy.sleep(1.5)
                        item_type = self.arm.grasp()
                        rospy.sleep(0.5)
                    if item_type:
                        break
                    else:
                        rospy.loginfo("========没扫描到，向前进一点=====")
                        rospy.sleep(0.5)
                        self.robot.step_go_pro(0.15)
                        rospy.sleep(1.5)
                        item_type = self.arm.grasp()
                        rospy.sleep(0.5)
                if item_type == 0 or item_type == 1:
                    if self.arm.complete[46] and self.arm.complete[88] and self.arm.complete[85]:
                        self.stop_flag = True
                        return
                    else:
                        sorting_name = "Sorting_DA"
                rospy.loginfo("========向后退一点=====")
                for _ in range(i + 3):
                    self.robot.step_go_pro(-0.15)  # 后退

            rospy.sleep(0.5)
            if self.stop_flag: return
            if item_type == 0:
                continue

            # ====放置物品====
            self.arm.arm_default_pose()
            rospy.loginfo("========前往放置区=====")
            ret = self.robot.goto_local(items_place_dict[item_type]) # 根据抓到的物品类型，导航到对应的放置区
            rospy.sleep(2.0) # 停稳

            if ret:
                pass
            else:
                rospy.logwarn("task error: navigation to the drop_place fails!!!")
                rospy.loginfo("continue to next task")
            self.arm.drop(item_type)
            self.robot.step_back(distance=0.4)
            if self.stop_flag:
                self.stop_flag = False

            # 下一步
            if items_place_dict[item_type] == "Collection_B":
                sorting_name = "Sorting_AB"
            elif items_place_dict[item_type] == "Collection_C":
                sorting_name = "Sorting_BC"
            elif items_place_dict[item_type] == "Collection_D":
                sorting_name = "Sorting_CD"


    def task_cmd_cb(self,flag):
        if flag:
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task)
                self.task_run_th.start()
                rospy.loginfo("start task!!!")
            else:
                rospy.logwarn("waiting for thread exit...")
                self.stop_flag = True
                self.task_run_th.join()
                rospy.logwarn("thread exit success")

    def grasp_cb(self, msg):
        if not self.task_run_th.is_alive():
            if msg.data == "1":
                self.arm.grasp()
            elif msg.data == "0":
                self.arm.drop(0)
                self.arm.arm_default_pose()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logdebug("Mark_move finished.")
