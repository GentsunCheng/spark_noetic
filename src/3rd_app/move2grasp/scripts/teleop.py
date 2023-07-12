#!/usr/bin/env python3
# -*- coding: utf-8 -*

import  os
import  sys
import threading
import  tty, termios
import roslib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# 全局变量
cmd = Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
grasp_pub = rospy.Publisher('/grasp', String, queue_size=10)

global can_grasp
global can_release

def grasp_status_cp(msg):
    global can_release,can_grasp
    # 物体抓取成功,让机器人回起始点
    if msg.data=='0':
        can_release=True
    if msg.data=='1' or msg.data=='-1':
        can_grasp=True
    if msg.data=='403':
        can_release=True
        can_grasp=True
grasp_status=rospy.Subscriber('/grasp_status', String, grasp_status_cp, queue_size=10)

def keyboardLoop():
    rospy.init_node('teleop')
    #初始化监听键盘按钮时间间隔
    rate = rospy.Rate(rospy.get_param('~hz', 1000))

    #速度变量
    # 慢速
    walk_vel_ = rospy.get_param('walk_vel', 0.2)
    # 快速
    run_vel_ = rospy.get_param('run_vel', 1.0)
    yaw_rate_ = rospy.get_param('yaw_rate', 1.0)
    yaw_rate_run_ = rospy.get_param('yaw_rate_run', 1.5)
    # walk_vel_前后速度
    max_tv = walk_vel_
    # yaw_rate_旋转速度
    max_rv = yaw_rate_
    # 参数初始化
    global can_release,can_grasp,speed_mod,can_run
    can_grasp=True
    can_release=False
    speed_mod=1

    print ("[Q] quit")
    print ("[WASD] move,[space] brake")
    print ("[.] switch speed mod")
    print ("[G] carry,[0] close pump")
    print ("[-]&[+] spare plan")
    print ("[1~3] switch step")
    print ("[R] default arm pose")

    #读取按键循环
    while not rospy.is_shutdown():
        # linux下读取键盘按键
        fd = sys.stdin.fileno()
        turn = 0.0
        speed = 0.0
        can_run = 1
        old_settings = termios.tcgetattr(fd)
		#不产生回显效果
        old_settings[3] = old_settings[3] & ~termios.ICANON & ~termios.ECHO
        try :
            tty.setraw( fd )
            print("\033[Kspeed: {}".format(speed_mod), end="\r")
            ch = sys.stdin.read( 1 )
        finally :
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        # ch代表获取的键盘按键

    # 抓取和释放
        if ch == 'g':
            if can_grasp:
                msg=String()
                msg.data='0'
                grasp_pub.publish(msg)
                can_grasp=False
                speed = 0
                turn = 0
        elif ch == 'h':
            if can_release:
                msg=String()
                msg.data = '1'
                grasp_pub.publish(msg)
                can_release=False
                speed = 0
                turn = 0
        elif ch =='0':
            if can_release:
                msg=String()
                msg.data = '10'
                grasp_pub.publish(msg)
                can_release=False
                speed = 0
                turn = 0

    # 备用状态
        elif ch=='+':
            msg=String()
            msg.data = '200'
            grasp_pub.publish(msg)
            can_release=False
            speed = 0
            turn = 0

    # 机械臂位姿调整
        # 三层位姿
        elif ch=='1':
            msg=String()
            msg.data = '11'
            grasp_pub.publish(msg)
            can_release=False
            speed = 0
            turn = 0
        elif ch=='2':
            msg=String()
            msg.data = '12'
            grasp_pub.publish(msg)
            can_release=False
            speed = 0
            turn = 0
        elif ch=='3':
            msg=String()
            msg.data = '13'
            grasp_pub.publish(msg)
            can_release=False
            speed = 0
            turn = 0

        # 默认位姿
        elif ch == 'r':
            msg=String()
            msg.data='403'
            grasp_pub.publish(msg)
            can_release=False
            speed = 0
            turn = 0

    # 第四关节调整
        elif ch == '4':
            msg=String()
            msg.data='41'
            grasp_pub.publish(msg)
        elif ch == '5':
            msg=String()
            msg.data='42'
            grasp_pub.publish(msg)
        elif ch == '6':
            msg=String()
            msg.data='43'
            grasp_pub.publish(msg)

    # 底盘行动控制
        # 刹车
        elif ch == ' ':
            stop_robot()
            can_run = 0

        # 模式切换
        elif ch=='.':
            speed=0
            turn=0
            if speed_mod==0:
                speed_mod=1
            elif speed_mod==1:
                speed_mod=0

        elif can_run:
            if ch == 'w':
                max_tv = walk_vel_
                if speed_mod==0:
                    speed = 0.2
                elif speed_mod==1:
                    speed = 1.3
                turn = 0
            elif ch == 's':
                max_tv = walk_vel_
                if speed_mod==0:
                    speed = -0.2
                elif speed_mod==1:
                    speed = -1.3
                turn = 0
            elif ch == 'a':
                max_tv = yaw_rate_
                if speed_mod==0:
                    turn = 0.2
                elif speed_mod==1:
                    turn = 1
                speed = 0
            elif ch == 'd':
                max_tv = yaw_rate_
                if speed_mod==0:
                    turn = -0.2
                elif speed_mod==1:
                    turn = -1
                speed = 0

    # 特殊状态
        # 机械臂复位
        elif ch == '`':
            msg=String()
            msg.data='404'
            grasp_pub.publish(msg)
            speed = 0
            turn = 0
        # 退出
        elif ch == 'q':
            exit()
        else:
            max_tv = walk_vel_
            max_rv = yaw_rate_
            speed = 0
            turn = 0

        #发送消息
        cmd.linear.x = speed * max_tv
        cmd.angular.z = turn * max_rv
        pub.publish(cmd)
        rate.sleep()

def stop_robot():
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    pub.publish(cmd)

if __name__ == '__main__':
    try:
        keyboardLoop()
    except rospy.ROSInterruptException:
        pass