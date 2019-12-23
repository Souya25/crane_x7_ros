#!/usr/bin/env python
# coding: utf-8
#シミュレータ用(つかむときのグリッパーの角度：23.5度)
#実機用(objectがやわらかいから10度)

import rospy
import time
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np

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
    
    def go(self):
        #search position
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
     
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint",
                                       "crane_x7_wrist_joint",]
      
        joint_values = [0, 0, 0, -1.570795, 0, -1.570795, 0]
       
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
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
