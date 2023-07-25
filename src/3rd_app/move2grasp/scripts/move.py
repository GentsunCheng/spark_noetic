#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
import tf
from actionlib_msgs.msg import *
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from math import radians, pi

class Move2Grasp():
    def __init__(self):
        rospy.init_node('move2grasp', anonymous=False)

        rospy.on_shutdown(self.shutdown)
        #订阅RVIZ上的点击事件
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        #订阅机械臂抓取状态
        # rospy.Subscriber('/grasp_status', String, self.grasp_status_cp, queue_size=10)
        # Publisher to manually control the robot (e.g. to stop it)
        # 发布TWist消息控制机器人
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # 发布机械臂抓取指令
        self.grasp_pub = rospy.Publisher('/grasp', String, queue_size=10)

        # Subscribe to the move_base action server
        # 订阅move_base服务器的消息
        self.move_base = actionlib.ActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait 60 seconds for the action server to become available
        # 等待move_base服务器建立
        self.move_base.wait_for_server(rospy.Duration(60))

        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")


    def cp_callback(self, msg):
        rospy.loginfo("POINT:%f,%f,%f", msg.point.x, msg.point.y, msg.point.z)
        # Initialize the waypoint goal
        goal = MoveBaseGoal()
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Set the goal position
        goal.target_pose.pose.position = Point(msg.point.x, msg.point.y, msg.point.z)

        # Get the current robot position and orientation using tf
        listener = tf.TransformListener()
        listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        # Set the current orientation as the goal orientation
        goal.target_pose.pose.orientation.x = rot[0]
        goal.target_pose.pose.orientation.y = rot[1]
        goal.target_pose.pose.orientation.z = rot[2]
        goal.target_pose.pose.orientation.w = rot[3]
        # Start the robot moving toward the goal
        self.move_base.send_goal(goal)
        # If we don't get there in time, abort the goal
        # 如果没有到达，修正朝向再发送
        for i in range(3):
            rospy.sleep(2)
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
            goal.target_pose.pose.orientation.x = rot[0]
            goal.target_pose.pose.orientation.y = rot[1]
            goal.target_pose.pose.orientation.z = rot[2]
            goal.target_pose.pose.orientation.w = rot[3]
            self.move_base.send_goal(goal)


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(1)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        Move2Grasp()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Move2grasp finished.")
