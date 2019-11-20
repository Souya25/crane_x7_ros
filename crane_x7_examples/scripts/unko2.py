#!/usr/bin/env python
# coding: utf-8
# Writen by Shota Hirama

import rospy
import time
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from control_msgs.msg import (      #Add 3lines
    GripperCommandAction,
    GripperCommandGoal
 )

from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys

"""
class GripperClient(object):  #Add class
    def __init__(self):
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()

        # Wait 5 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(5.0))
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()
"""

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
        #self.gripperclear()
    """ 
    def gripper_command(self):
        position = math.radians(45.0)
        effrt  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
        self.gripper_client.send_goal(self._goal,feedback_cb=self.feedback)
        time.sleep(1)
    """
    def go(self):
        
          
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
        #self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        #time.sleep(1)
       
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint", "crane_x7_shoulder_revolute_part_tilt_joint",
                                       "crane_x7_upper_arm_revolute_part_twist_joint", "crane_x7_upper_arm_revolute_part_rotate_joint",
                                       "crane_x7_lower_arm_fixed_part_joint", "crane_x7_lower_arm_revolute_part_joint",
                                       "crane_x7_wrist_joint",]
        
       
       
        point = JointTrajectoryPoint()
  
        joint_values = [0.21304366673905673, 0.02974849165462512, -0.20838138537767303, -2.197273761794529, -0.00897299687593378, -0.9915954713398651, -1.5583097453428332]
       
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=1.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        print("wait start")  
        time.sleep(0.0)
        print("wait end")  

        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)

        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        #return self._client.get_result()
        
        
        
        point = JointTrajectoryPoint()
        
        joint_values = [0.1721327091256155, -0.3630645331662308, -0.1862058852524493, -2.3243098124547927, 0.130925888195077, -0.48587059161714485, -1.6908899687220424]
       
        position = math.radians(45.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=4.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        print("wait start")  
        time.sleep(0.0)
        print("wait end")  
        print(point)
        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        return self._client.get_result()
        
        """ 
        gc = GripperClient()
         # Open grippers(45degrees)
        print "Open Gripper."
        gripper = 45.0
        gc.command(math.radians(gripper),1.0)
        result = gc.wait(2.0)
        print result
        time.sleep(1)
        print ""
        """  
        #goal.trajectory.points.append(point)
        #self._client.send_goal(goal)

        
        """joint_values = [-30.0, -10.0, -30.0, 0.0, -20.0]
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
        """

    def feedback(self,msg):
        print("feedback callback")
        print(msg)



if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    result = arm_joint_trajectory_example.go()
    print(result)
