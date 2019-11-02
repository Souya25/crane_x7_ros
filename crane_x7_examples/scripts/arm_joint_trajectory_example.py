#!/usr/bin/env python
# coding: utf-8
# Writen by Shota Hirama

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys


class ArmJointTrajectoryExample(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)

    def move_arm(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint", "crane_x7_wrist_joint"]

        joint_values = [0.0, 10.0, 20.0, 30.0, 40.0]
        joint_values.extend(reversed(joint_values))
        for i, p in enumerate(joint_values):
            point = JointTrajectoryPoint()
            for _ in range(len(goal.trajectory.joint_names)):
                point.positions.append(math.radians(p))
            point.time_from_start = rospy.Duration(secs=i)
            goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        return self._client.get_result()


if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    result = arm_joint_trajectory_example.move_arm()
    print(result)
