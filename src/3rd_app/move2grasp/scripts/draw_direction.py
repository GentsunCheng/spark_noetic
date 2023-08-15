#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped


class LineDrawer:
    def __init__(self):
        rospy.init_node("draw_direction")
        self.marker_pub = rospy.Publisher(
            "draw_direction", Marker, queue_size=10)
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        self.rate = rospy.Rate(10)  # Publish rate

        self.speed_mod = 0.0

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
        marker.color.r = self.speed_mod
        marker.color.g = not self.speed_mod
        marker.color.b = 0.0
        marker.color.a = (not self.speed_mod) / 4.0 + 0.5

        # Define line points
        point1 = Point()
        point1.x = 5.5
        point1.y = 2.25
        point1.z = 0.0

        point2 = Point()
        point2.x = 7.0
        point2.y = 2.25
        point2.z = 0.0

        point3 = Point()
        point3.x = 7.0
        point3.y = 0.75
        point3.z = 0.0

        point4 = Point()
        point4.x = 5.5
        point4.y = 0.75
        point4.z = 0.0

        point5 = Point()
        point5.x = 6.0
        point5.y = 1.75
        point5.z = 0.0

        point6 = Point()
        point6.x = 6.5
        point6.y = 1.75
        point6.z = 0.0

        point7 = Point()
        point7.x = 6.5
        point7.y = 1.25
        point7.z = 0.0

        point8 = Point()
        point8.x = 6.0
        point8.y = 1.25
        point8.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)
        marker.points.append(point4)
        marker.points.append(point1)
        marker.points.append(point5)
        marker.points.append(point6)
        marker.points.append(point7)
        marker.points.append(point8)
        marker.points.append(point5)

        self.marker_pub.publish(marker)

    def cp_callback(self, msg):
        if msg.point.x > 8.0 and msg.point.x < 8.5 and msg.point.y > - 0.25 and msg.point.y < 0.25:
            if self.speed_mod:
                self.speed_mod = 0.0
            else:
                self.speed_mod = 1.0

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
