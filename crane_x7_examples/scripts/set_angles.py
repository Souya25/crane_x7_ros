#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler

rospy.init_node("pose_groupstate_example")
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm")
arm.set_max_velocity_scaling_factor(0.1)
gripper = moveit_commander.MoveGroupCommander("gripper")

while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    rospy.sleep(1.0)

line = ""
while "end" not in line:
    print("input angle (or type 'end' for finishing)")
    print("example1: a 0 60 (60[deg] for arm joint 0)")
    print("example2: g 0 30 (30[deg] for gripper joint 0)")
    print("------------")
    line = sys.stdin.readline()
    inp = line.split()
    try:
        part = inp[0]
        joint = int(inp[1])
        angle = float(inp[2])/180.0*math.pi

        print(part, joint, angle)
        if part == "a":
            arm_joint_values = arm.get_current_joint_values()
            arm_joint_values[joint] = angle
            arm.set_joint_value_target(arm_joint_values)
            arm.go()		
        elif part == "g":
            gripper_joint_values = gripper.get_current_joint_values()
            gripper_joint_values[joint] = angle
            gripper.set_joint_value_target(gripper_joint_values)
            gripper.go()		

    except:
        continue

    line = sys.stdin.readline()

# 移動後の手先ポーズを表示
arm_goal_pose = arm.get_current_pose().pose
print("Arm goal pose:")
print(arm_goal_pose)
print("done")

