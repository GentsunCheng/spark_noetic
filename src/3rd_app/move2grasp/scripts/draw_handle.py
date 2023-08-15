#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PointStamped


class LineDrawer:
    def __init__(self):
        rospy.init_node("draw_handle")
        self.marker_pub = rospy.Publisher(
            "draw_handle", Marker, queue_size=10)
        rospy.Subscriber('clicked_point', PointStamped, self.cp_callback)
        self.rate = rospy.Rate(10)  # Publish rate

        pointstamped_t = PointStamped()
        self.point_c = pointstamped_t.point
        self.point_t = pointstamped_t.point
        self.speed_mod = 0.0
        self.point_t.x = 6.25
        self.point_t.y = 1.5
        self.point_t.z = 0.0
        self.point_c = self.point_t
        self.move_mod = 1.0

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
        marker.color.a = self.move_mod * ((not self.speed_mod) / 4.0 + 0.5)

        point1 = Point()
        point1.x = self.point_c.x - 0.125
        point1.y = self.point_c.y + 0.125
        point1.z = 0.0

        point2 = Point()
        point2.x = self.point_c.x + 0.125
        point2.y = self.point_c.y + 0.125
        point2.z = 0.0

        point3 = Point()
        point3.x = self.point_c.x + 0.125
        point3.y = self.point_c.y - 0.125
        point3.z = 0.0

        point4 = Point()
        point4.x = self.point_c.x - 0.125
        point4.y = self.point_c.y - 0.125
        point4.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)
        marker.points.append(point4)
        marker.points.append(point1)

        self.marker_pub.publish(marker)

    def cp_callback(self, msg):
        self.move_mod = msg.point.z
        if msg.point.x > 5.5 and msg.point.x < 7.0 and msg.point.y > 0.75 and msg.point.y < 2.25:
            self.point_c = msg.point
            rospy.sleep(0.3)
            self.point_c = self.point_t
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