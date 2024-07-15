#!/usr/bin/python3

import sys
import os
import yaml
import threading

import rospy
import rospkg
import actionlib
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
        self.arm_position_slow_pub = rospy.Publisher(
            "position_slow_topic", swiftpro.msg.position, queue_size=1)
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

    def set_pose_slow(self, x, y, z):
        pos = position()
        pos.x = x
        pos.y = y
        pos.z = z
        pos.speed = 50
        self.arm_position_slow_pub.publish(pos)
        

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
        # 获取标定文件相关信息
        rospack = rospkg.RosPack()
        package_path = os.path.join(rospack.get_path('auto_match'))          # 获取功能包路径
        items_path = os.path.join(package_path, 'config', 'items_config.yaml')  # 获取物体标签路径
        try:
            with open(items_path, "r", encoding="utf8") as f:
                items_content = yaml.load(f.read(), Loader=yaml.FullLoader)
        except Exception:
            rospy.logerr("can't not open file")
            sys.exit(1)
        if isinstance(items_content, type(None)):
            rospy.logerr("items file empty")
            sys.exit(1)

        # 根据yaml文件，确定抓取物品的id号
        self.search_id = [
            items_content["items"][items_content["objects"]["objects_a"]],
            items_content["items"][items_content["objects"]["objects_b"]],
            items_content["items"][items_content["objects"]["objects_c"]]
        ]

    def detector(self):
        '''
        获取需要抓取的物品在显示屏上的坐标位置
        @return: 需要抓取的物品列表cube_list

        cube_list[i]:代表第几个物体
        cube_list[i][0]:代表第i个物体的ID信息;               cube_list[i][1]:代表第i个物体的位置信息
        cube_list[i][1][1]:代表第i个物体的x方向上的位置;     cube_list[i][1][0]:代表第i个物体的y方向上的位置
        '''
        obj_dist = {}
        cube_list = []
        obj_array = None

        try:
            obj_array = rospy.wait_for_message(
                "/objects", Detection2DArray, timeout=5)
        except Exception:
            cube_list.clear()
            return cube_list
        
        # 提取
        for obj in obj_array.detections:
            obj_dist[obj.results[0].id] = [obj.bbox.center.x, obj.bbox.center.y, 0]

        # 筛选出需要的物品 cube_list中的key代表识别物体的ID，value代表位置信息
        for key, value in obj_dist.items():
            if key in self.search_id:
                cube_list.append([key, value])

        # 按识别次数排列并选择次数最高的物品
        # cube_list.sort(key=lambda x: x[1][2], reverse=True)
        return cube_list

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
            # print("cube_list:",cube_list)

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
                print("finded 3 objects")
                # 根据 cube_list[i][1][0] 的值对 cube_list 进行排序
                sorted_cube_list = sorted(cube_list, key=lambda x: x[1][0], reverse=False)

                # 建立对应关系 sorted_cube_list[0][0]代表识别到的物体ID
                items_place_dict[sorted_cube_list[0][0]] = "Collection_B" # 最左边的物体对应收取区D
                items_place_dict[sorted_cube_list[1][0]] = "Collection_C" # 中间的物体对应收取区C
                items_place_dict[sorted_cube_list[2][0]] = "Collection_D" # 最右边的物体对应收取区B
                
                print("readying to turn back")
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

        # 获取标定文件数据
        filename = os.environ['HOME'] + "/thefile.txt"
        with open(filename, 'r') as f:
            s = f.read()
        arr = s.split()
        self.x_kb = [float(arr[0]), float(arr[1])]
        self.y_kb = [float(arr[2]), float(arr[3])]        

        # 创建机械臂控制接口的对象
        self.interface = SwiftProInterface()

        self.grasp_status_pub = rospy.Publisher("/grasp_status", String, queue_size=1)

        self.height, self.width = 480, 640
        self.center_x = self.width // 2  # 图像底边中心的x坐标
        self.bottom_y = self.height  # 图像底边的y坐标

        self.time = {46: 0, 88: 0, 85: 0}
        self.block_height = 100
        self.is_catched = False
        self.testing = False
 
    
    def grasp(self):
        '''
        使用深度学习找到所需物品在图像上的位置, 估算物品实际位置, 让机械臂抓取
        @return: 抓取到物品的id, 0为未识别到需要的物品
        '''
        x = 0
        y = 0
        z = 0

        # 寻找物品
        cube_list = self.cam.detector()
        # rospy.sleep(2)
        if len(cube_list) == 0:
            rospy.logwarn("没有找到物品啊。。。去下一个地方")
            self.grasp_status_pub.publish(String("1"))
            return 0

        closest_x = cube_list[0][1][1]
        closest_y = cube_list[0][1][0]
        id = cube_list[0][0]

        for pice in cube_list:
            x = pice[1][1]
            y = pice[1][0]
            if y > closest_y:
                closest_x = x
                closest_y = y
                id = pice[0]

        # 获取机械臂目标位置
        x = self.x_kb[0] * closest_x + self.x_kb[1]
        y = self.y_kb[0] * closest_y + self.y_kb[1]
        z = -50.0

        print(f"找到物品了！它在: {x}, {y}, {z}")

        # 机械臂移动到目标位置上方
        self.interface.set_pose(x, y, z + 20)
        rospy.sleep(0.2)

        self.interface.set_pose(x, y, z)
        # 打开气泵，进行吸取
        self.interface.set_pump(True)
        rospy.sleep(0.5)

        # 抬起检测
        self.interface.set_pose(x, y, 60)
        rospy.sleep(1.0)
        self.sub_tmp = rospy.Subscriber('/scan', LaserScan, self.check_grasp_state)
        self.testing = True

        while self.testing:
            pass

        if self.is_catched:
            # 抬起目标方块
            print(f"我把物品抬起来了")
            self.interface.set_pose(x, y, z + 120)
            rospy.sleep(0.5)
            self.interface.set_pose_slow(10, 150, 175)

            self.grasp_status_pub.publish(String("0"))
            self.time[id] = self.time[id] + 1
            return id
        else:
            self.interface.set_pump(False)
            self.arm_grasp_ready()
            return 0

    def drop(self, item):
        '''
        放置方块, 可以先判断是否有方块, 从而调整放置高度
        @param check: 是否判断有无方块, 默认判断
        @return item_id: 执行结果
        '''
        x = 280
        y = 0
        z = 175
        self.interface.set_pose(x, y, z)
        rospy.sleep(1.5)
        z = -125 + self.time[item] * self.block_height if self.time[item] < 3 else -125 + 3 * self.block_height
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
        rospy.sleep(0.3)

        # 移动到其他地方
        x, y = 50, 150
        self.interface.set_pose(x, y, z)
        rospy.sleep(0.2)
        self.arm_grasp_ready()

        self.grasp_status_pub.publish(String("0"))

        return True
    
    # 检查是否成功抓取，成功返回True，反之返回False
    def check_grasp_state(self, data):
        self.sub_tmp.unregister()
        rospy.sleep(0.3)
        for distance in data.ranges:
            if distance < 0.3:
                self.is_catched = True
        if not self.is_catched:
            sub = rospy.Publisher("armreset", String, queue_size=1)
            sub.publish("reset")
            sub.unregister()
        self.testing = False

        

    def arm_position_reset(self):
        '''
        校准机械臂的坐标系, 机械臂因碰撞导致坐标计算出问题时使用
        '''
        r1 = rospy.Rate(10)
        self.interface.set_status(False)
        r1.sleep()
        self.interface.set_status(True)
        r1.sleep()

    def arm_home(self, block=False):
        '''
        收起机械臂(无物品)
        '''
        self.interface.set_pose(130, 0, 35)
        if block:
            rospy.sleep(1.0)

    def arm_grasp_ready(self, block=False):
        '''
        移动机械臂到摄像头看不到的地方，以方便识别与抓取
        '''
        self.interface.set_pose(10, 150, 160)
        if block:
            rospy.sleep(1.0)

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

        # 创建根据定位码调整spark移动到台前的action客户端
        # self.tag_adjustment_client = actionlib.SimpleActionClient(
        #     'spark_alignment_mark', TagAdjustmentAction)
        # self.tag_adjustment_client.wait_for_server()

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
    
    def step_go(self,dis):
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

    def step_go_pro(self, value):
        twist_go = Twist()
        twist_go.linear.x = value
        self.cmd_pub.publish(twist_go)


class AutoAction:
    def __init__(self):
        # # 初始化节点
        # if init_node:
        rospy.init_node('spark_auto_match_node', anonymous=True)

        print("========ready to task===== ")

        # 实例化Cam
        try: self.cam = CamAction()
        except Exception as e:  print("except cam:",e)
        print("========实例化Cam===== ")
        # 实例化Arm
        try: self.arm = ArmAction()
        except Exception as e:  print("except arm:",e)
        print("========实例化Arm===== ")
        # 实例化Robot
        try: self.robot = RobotMoveAction()
        except Exception as e:  print("except robot:",e)
        print("========实例化Robot===== ")

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
        self.arm.arm_position_reset()  # 重置机械臂坐标系
        self.arm.arm_grasp_ready()  # 移动机械臂到其他地方

        

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
                    self.robot.step_go_pro(0.2)
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


            if ret: # 判断是否成功到达目标点
                print("========往前走看清一点===== ")
                # self.robot.step_go(0.02)  # 前进
                # self.robot.step_go(value)   ############################################
                rospy.sleep(0.5) # 停稳
                print("========扫描中，准备抓取===== ")
                item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型
                for i in range(3):
                    if item_type:
                        break
                    else:
                        print("========没抓到，向前进一点===== ")
                        self.robot.step_go(0.05)
                        rospy.sleep(1.0)
                        item_type = self.arm.grasp()
                print("========向后退一点===== ")
                for _ in range(i + 3):
                    self.robot.step_back(0.05)  # 后退

                rospy.sleep(0.5)

                grasp_ctrl_state = True  # ==========================================
                print("grasp_:", grasp_ctrl_state)
                #rospy.sleep(4)
                if grasp_ctrl_state != True:
                    self.arm.arm_grasp_ready()
                    self.robot.step_go(0.23)
                    rospy.sleep(1.5) # 停稳
                    item_type = self.arm.grasp()  # 抓取物品并返回抓取物品的类型
                    if item_type != 0:         # 新增##############################################################################
                        grasp_ctrl_state = True
                    self.robot.step_back()  # 后退
                

                if self.stop_flag: return

            if item_type == 0:
                continue

            if grasp_ctrl_state != False:
                # ====放置物品====
                self.arm.arm_grasp_ready()
                print("========前往放置区===== ")
                ret = self.robot.goto_local(items_place_dict[item_type]) # 根据抓到的物品类型，导航到对应的放置区
                rospy.sleep(2.0) # 停稳
                if self.stop_flag: return

                if ret: 
                    self.arm.drop(item_type)  # 放下物品
                    self.robot.step_back(distance=0.4)  # 后退
                    if self.stop_flag: return
                else:
                    rospy.logerr("task error: navigation to the drop_place fails")
                    self.arm.drop(item_type)
                if self.stop_flag: return

                # 下一步
            if items_place_dict[item_type] == "Collection_B":
                sorting_name = "Sorting_AB"
            elif items_place_dict[item_type] == "Collection_C":
                sorting_name = "Sorting_BC"
            elif items_place_dict[item_type] == "Collection_D":
                sorting_name = "Sorting_CD"


    def task_cmd_cb(self,flag):
        if flag :
            if not self.task_run_th.is_alive():
                self.stop_flag = False
                self.task_run_th = threading.Thread(target=self.run_task, args=())
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
                self.arm.arm_grasp_ready()
            else:
                rospy.logwarn("grasp msg error")


if __name__ == '__main__':
    try:
        AutoAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Mark_move finished.")
