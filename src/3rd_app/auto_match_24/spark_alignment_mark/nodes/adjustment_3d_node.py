#!/usr/bin/python3

import math
import rospy
import actionlib
from spark_alignment_mark.msg import TagAdjustmentAction, TagAdjustmentGoal, TagAdjustmentResult
from common.msg import MoveStraightDistanceAction, TurnBodyDegreeAction, MoveStraightDistanceGoal, TurnBodyDegreeGoal
import tf
import tf.transformations
from simple_pid import PID

class SparkActionMotionCtrl:
    def __init__(self, dead_distance=0.05) -> None:
        self.DEAD_DISTANCE = dead_distance

        self.move_action_cli = actionlib.SimpleActionClient('move_straight', MoveStraightDistanceAction)
        self.move_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(5))

        self.turn_action_cli = actionlib.SimpleActionClient('turn_body', TurnBodyDegreeAction)
        self.turn_action_cli.wait_for_server(timeout=rospy.Duration.from_sec(5))

        self.pid_lin = PID(1.0, 0.1, 0.05, setpoint=0)
        self.pid_ang = PID(1.0, 0.1, 0.05, setpoint=0)

    def move_straight(self, distance, timeout=30, useLidar=False):
        goal = MoveStraightDistanceGoal(
            type=MoveStraightDistanceGoal.TYPE_SCAN if useLidar else MoveStraightDistanceGoal.TYPE_ODOM,
            move_distance=distance
        )

        self.move_action_cli.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout))
        return self.move_action_cli.get_result()

    def turn_body(self, degree, timeout=30):
        degree = (degree + 360) % 360
        if degree > 180:
            degree -= 360

        goal = TurnBodyDegreeGoal(goal_degree=degree)
        self.turn_action_cli.send_goal_and_wait(goal, rospy.Duration.from_sec(timeout))
        return self.turn_action_cli.get_result()

    def to_pose(self, px, py, angle):
        target_angle = math.degrees(math.atan2(py, px))
        distance = math.sqrt(px**2 + py**2)

        if abs(distance) > self.DEAD_DISTANCE:
            self.turn_body(target_angle)
            self.move_straight(distance)
            self.turn_body(angle - target_angle)
        else:
            self.turn_body(angle)

        return True

    def stop(self):
        self.move_action_cli.cancel_all_goals()
        self.turn_action_cli.cancel_all_goals()


class ARTagTool:
    def __init__(self) -> None:
        self.listener = tf.TransformListener()

    def get_tag(self, id):
        try:
            (target_trans, target_rot) = self.listener.lookupTransform('/base_link', f'/ar_marker_{id}', rospy.Time(0))
            return tf.transformations.compose_matrix(translate=target_trans, angles=tf.transformations.euler_from_quaternion(target_rot))
        except:
            return None


class SparkAdjustment:
    def __init__(self, left_id=21, center_id=22, right_id=23, tag_distance=0.2):
        self.LEFT_ID = left_id
        self.CENTER_ID = center_id
        self.RIGHT_ID = right_id
        self.TAG_DISTANCE = tag_distance

        self.spark_motion = SparkActionMotionCtrl()
        self.ar_tag = ARTagTool()

        self.broadcaster = tf.TransformBroadcaster()

        self.action_server = actionlib.SimpleActionServer('spark_alignment_mark', TagAdjustmentAction, self.execute, auto_start=False)
        self.action_server.register_preempt_callback(self.preemptCB)
        self.action_server.start()

    def face_and_center(self, axis, negative=False, target_distance=0.4):
        tag_c = self.ar_tag.get_tag(self.CENTER_ID)
        if tag_c is None:
            rospy.logwarn("can not found center tag.")
            return False

        px, py = tag_c[:2, 3]

        if axis == 'x':
            target_x = px + (target_distance if not negative else -target_distance)
            target_y = py
        elif axis == 'y':
            target_x = px
            target_y = py + (target_distance if not negative else -target_distance)
        else:
            rospy.logerr("axis error! please check your code.")
            return False

        target_angle = math.degrees(math.atan2(target_y - py, target_x - px))
        if negative:
            target_angle += 180
            if target_angle > 180:
                target_angle -= 360

        self.spark_motion.to_pose(target_x - px, target_y - py, target_angle)
        return True

    def preemptCB(self):
        self.spark_motion.stop()
        self.action_server.set_aborted()

    def execute(self, goal: TagAdjustmentGoal):
        success = self.face_and_center(goal.target_axis, goal.negative, goal.target_distance)
        result = TagAdjustmentResult()
        result.result = success
        self.action_server.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node('spark_adjustment_server')
    server = SparkAdjustment()
    rospy.spin()
