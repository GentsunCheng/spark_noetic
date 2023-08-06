#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped

class LineDrawer:
    def __init__(self):
        rospy.init_node("draw_step")
        self.marker_pub = rospy.Publisher("draw_step", Marker, queue_size=10)
        self.rate = rospy.Rate(10)  # Publish rate

    def draw_line(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.01  # Line width

        # Set line color (RGBA, 0-1)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        point1 = Point()
        point1.x = 7.0
        point1.y = 2.25
        point1.z = 0.0

        point2 = Point()
        point2.x = 7.5
        point2.y = 2.25
        point2.z = 0.0

        point3 = Point()
        point3.x = 7.5
        point3.y = 0.75
        point3.z = 0.0

        point4 = Point()
        point4.x = 7.0
        point4.y = 0.75
        point4.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)
        marker.points.append(point4)
        marker.points.append(point1)

        self.marker_pub.publish(marker)

    def run(self):
        while not rospy.is_shutdown():
            self.draw_line()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        line_drawer = LineDrawer()
        line_drawer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Draw finished.")