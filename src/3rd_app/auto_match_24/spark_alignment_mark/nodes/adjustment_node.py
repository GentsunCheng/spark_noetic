#!/usr/bin/python3

import rospy
import math
import cmath
import actionlib
from actionlib_msgs.msg import GoalStatus
from spark_alignment_mark.msg import TagAdjustmentAction, TagAdjustmentGoal, TagAdjustmentFeedback, TagAdjustmentResult
from ar_track_alvar_msgs.msg import AlvarMarkers
import common.msg
import common.srv

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, current_value):
        error = self.setpoint - current_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class SparkAdjustment:
    def __init__(self):
        self.left_id = 21
        self.center_id = 22
        self.right_id = 23

        self.move_action_cli = actionlib.SimpleActionClient('move_straight', common.msg.MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(3))

        self.turn_action_cli = actionlib.SimpleActionClient('turn_body', common.msg.TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(3))

        self.action_server = actionlib.SimpleActionServer('spark_alignment_mark', TagAdjustmentAction, self.execute, auto_start=False)
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

        self.angle_pid = PIDController(1.0, 0.01, 0.1)  # PID参数需要根据实际调整
        self.distance_pid = PIDController(1.0, 0.01, 0.1)  # PID参数需要根据实际调整

    def move_straight(self, distance, vel=0.1, timeout=30, block=True, useLidar=False):
        goal = common.msg.MoveStraightDistanceGoal(
            type=common.msg.MoveStraightDistanceGoal.TYPE_SCAN if useLidar else common.msg.MoveStraightDistanceGoal.TYPE_ODOM,
            move_distance=distance,
            const_rot_vel=vel
        )
        if block:
            self.move_action_cli.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout))
        else:
            self.move_action_cli.send_goal(goal)
        return self.move_action_cli.get_result()

    def turn_body(self, degree, vel=0.25, timeout=30, block=True, is_const=True):
        degree = (degree + 180) % 360 - 180
        goal = common.msg.TurnBodyDegreeGoal(
            is_const_vel=is_const,
            goal_degree=degree,
            const_rot_vel=vel
        )
        if block:
            self.turn_action_cli.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout))
        else:
            self.turn_action_cli.send_goal(goal)
        return self.turn_action_cli.get_result()

    def execute(self, goal: TagAdjustmentGoal):
        self.action_server.publish_feedback(TagAdjustmentFeedback(state=TagAdjustmentFeedback.RUNNING))
        try:
            markers = rospy.wait_for_message("marker_center", AlvarMarkers, timeout=1).markers
        except rospy.ROSException:
            self.action_server.set_succeeded(TagAdjustmentResult(result=TagAdjustmentResult.ERROR))
            return

        if len(markers) < 2:
            rospy.logwarn("No enough tags found")
            self.action_server.set_succeeded(TagAdjustmentResult(result=TagAdjustmentResult.ERROR))
            return

        id_list = [x.id for x in markers]
        delta_x = markers[0].center.x - markers[1].center.x
        delta_y = (480 - markers[0].center.y) - (480 - markers[1].center.y)

        line_degrees = math.degrees(math.atan2(delta_y, delta_x))

        if goal.adjustment_type == TagAdjustmentGoal.TYPE_JUST_FACE:
            self.turn_body(self.angle_pid.compute(line_degrees))
        else:
            if self.center_id in id_list:
                mid_x = markers[id_list.index(self.center_id)].center.x
                mid_y = markers[id_list.index(self.center_id)].center.y
            else:
                mid_x = (markers[0].center.x + markers[1].center.x) / 2
                mid_y = 480 - (markers[0].center.y + markers[1].center.y) / 2

            mid_p, mid_theta = cmath.polar(complex(mid_x - 320, mid_y))
            mid_theta = math.degrees(mid_theta)
            turn_degrees = line_degrees + 90 if line_degrees < 0 else line_degrees - 90
            walk_distance = 0.0016 * (math.sin(math.radians(mid_theta - turn_degrees)) * mid_p)

            self.action_server.publish_feedback(TagAdjustmentFeedback(state=TagAdjustmentFeedback.EXEC_TURNNING))
            self.turn_body(self.angle_pid.compute(turn_degrees))

            self.action_server.publish_feedback(TagAdjustmentFeedback(state=TagAdjustmentFeedback.EXEC_MOVING))
            self.move_straight(self.distance_pid.compute(walk_distance))

            self.action_server.publish_feedback(TagAdjustmentFeedback(state=TagAdjustmentFeedback.EXEC_TURNNING))
            self.turn_body(self.angle_pid.compute(-90 if line_degrees < 0 else 90))

        self.action_server.set_succeeded(TagAdjustmentResult(result=TagAdjustmentResult.SUCCESS))

    def preemptCB(self):
        self.turn_body(degree=0, vel=0.05)
        self.move_straight(distance=0, vel=0.1)

if __name__ == '__main__':
    rospy.init_node('spark_alignment_mark', anonymous=True)
    adj = SparkAdjustment()
    rospy.spin()
