#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class LineDrawer:
    def __init__(self):
        rospy.init_node("draw_release")
        self.marker_pub = rospy.Publisher("draw_release", Marker, queue_size=10)
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
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.75
        marker.color.a = 1.0

        point1 = Point()
        point1.x = 8.25
        point1.y = 2.75
        point1.z = 0.0

        point2 = Point()
        point2.x = 7.75
        point2.y = 2.75
        point2.z = 0.0

        point3 = Point()
        point3.x = 7.75
        point3.y = 3.25
        point3.z = 0.0

        point4 = Point()
        point4.x = 8.75
        point4.y = 3.25
        point4.z = 0.0

        point5 = Point()
        point5.x = 8.75
        point5.y = 2.75
        point5.z = 0.0

        point6 = Point()
        point6.x = 8.25
        point6.y = 3.25
        point6.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)
        marker.points.append(point4)
        marker.points.append(point5)
        marker.points.append(point1)
        marker.points.append(point6)

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