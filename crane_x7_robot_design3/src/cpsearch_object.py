#!/usr/bin/env python
# coding: utf-8
#searching pose

import rospy
import time
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from geometry_msgs.msg import Point, Pose
import moveit_commander
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np
from geometry_msgs.msg import Pose2D

def yaw_of(object_orientation):                                                                                                           
     # クォータニオンをオイラー角に変換しyaw角度を返す
     euler = euler_from_quaternion(
         (object_orientation.x, object_orientation.y,
         object_orientation.z, object_orientation.w))
 
     return euler[2]
 
 

class ArmJointTrajectoryExample(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)

        self.gripper_client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.gripper_goal = GripperCommandGoal()
        # Wait 5 Seconds for the gripper action server to start or exit
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
           
        global sum_x
        sum_x = 0
        global sum_y
        sum_y = 0
        global count
        count = 0

    def callback(self, ms):
        #print("x座標 = {}".format(ms.x))
        #print("y座標 = {}".format(ms.y))
        #print("----------")

        if count < 100:
            global sum_x
            sum_x += ms.x
            global sum_y
            sum_y += ms.y
            global count
            count += 1

        else:
            #self.calculate作る(x,yを渡して計算する)
            self.go() 



    

        
        print("x座標 = {}".format(sum_x/count))
        print("y座標 = {}".format(sum_y/count))
        print("count = {}".format(count))
        print("x合計 = {}".format(sum_x))
        print("y合計 = {}".format(sum_y))
        print("----------")


    def pick(self, )
        target_pose = Pose()
        target_pose.position.x = object_position.x
        target_pose.position.y = object_position.y
        target_pose.position.z = APPROACH_Z
        q = quaternion_from_euler(-math.pi, 0.0, object_yaw)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        if arm.go() is False:
            print "Failed to approach an object."
            continue
        rospy.sleep(1.0)
        
    def go(self):
        #search position
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
     
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint",
                                       "crane_x7_wrist_joint",]
      
        joint_values = [0, 0, 0, -1.57, 0, -1.57, 1.57]
        print(joint_values) 
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=3.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)

        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

         
    def feedback(self,msg):
        print("feedback callback")

if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    arm_joint_trajectory_example.go()
