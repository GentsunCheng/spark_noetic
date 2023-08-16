#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped

class LineDrawer:
    def __init__(self):
        rospy.init_node("draw_step")
        self.marker_pub = rospy.Publisher("draw_step", Marker, queue_size=10)
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        self.rate = rospy.Rate(10)  # Publish rate
        self.step_mod = 0.0

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
        marker.color.r = 0.5
        marker.color.g = not self.step_mod
        marker.color.b = self.step_mod
        marker.color.a = 0.75

        point1 = Point()
        point1.x = 8.0
        point1.y = 2.25
        point1.z = 0.0

        point2 = Point()
        point2.x = 8.5
        point2.y = 2.25
        point2.z = 0.0

        point3 = Point()
        point3.x = 8.5
        point3.y = 0.75
        point3.z = 0.0

        point4 = Point()
        point4.x = 8.0
        point4.y = 0.75
        point4.z = 0.0

        point5 = Point()
        point5.x = 8.0
        point5.y = 0.25
        point5.z = 0.0

        point6 = Point()
        point6.x = 8.5
        point6.y = 0.25
        point6.z = 0.0

        point7 = Point()
        point7.x = 9.0
        point7.y = 0.75
        point7.z = 0.0

        point8 = Point()
        point8.x = 9.0
        point8.y = 0.25
        point8.z = 0.0

        point9 = Point()
        point9.x = 7.5
        point9.y = 0.25
        point9.z = 0.0

        point10 = Point()
        point10.x = 7.5
        point10.y = 0.75
        point10.z = 0.0

        marker.points.append(point4)
        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)
        marker.points.append(point4)
        marker.points.append(point5)
        marker.points.append(point6)
        marker.points.append(point3)
        marker.points.append(point7)
        marker.points.append(point8)
        marker.points.append(point9)
        marker.points.append(point10)
        marker.points.append(point4)

        self.marker_pub.publish(marker)

    def cp_callback(self, msg):
        if msg.point.x > 8.0 and msg.point.x < 8.5 and msg.point.y > 0.25 and msg.point.y < 0.75:
            if self.step_mod:
                self.step_mod = 0.0
            else:
                self.step_mod = 1.0
        elif (msg.point.x > 8.5 and msg.point.x < 9.0 and msg.point.y > 1.25 and msg.point.y < 1.75) or (msg.point.x > 7.5 and msg.point.x < 8.0 and msg.point.y > 1.25 and msg.point.y < 1.75) or (msg.point.x > 8.25 and msg.point.x < 8.75 and msg.point.y > 2.25 and msg.point.y < 2.75):
            self.step_mod = 0.0

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