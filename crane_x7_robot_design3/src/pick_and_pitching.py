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
import geometry_msgs.msg 
import moveit_commander
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion 
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np
from geometry_msgs.msg import Pose2D


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
       
        self.search_num = 0
        self.x = 0 
        self.y = 0 
        self.z = 0
        self.count=0
        self.tar_x = 0
        self.tar_y = 0
        self.mode = 0
        self.kaisuu =0
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        #self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
         
    global sum_x
    sum_x = 0
    global sum_y
    sum_y = 0
           
    def callback(self, ms):
        print("x :{}".format(ms.x)) 
        print("y :{}".format(ms.y))
        print("theta :{}".format(ms.theta)) 
        print("search_num : {}".format(self.search_num))
        if ms.theta == 1: #ms.theta == 1 no notoki kiiro naiyo 
                
            if self.count < 200:
                global sum_x
                sum_x += ms.x #軸変換(センサのy軸をマニピュレータのx軸に合わせる)
                global sum_y
                sum_y += ms.y #軸変換(センサのx軸をマニピュレータのy軸に合わせる)
                self.count += 1
            elif self.count == 200:
                print("ave_x :{}".format(sum_x/self.count))
                print("ave_y :{}".format(sum_y/self.count))
                self.mode = ms.theta
                self.tar_x = (sum_x/self.count)
                self.tar_y = (sum_y/self.count)
                self.count += 1
                print("already get data")
                #self.calculate(sum_x/self.count,sum_y/self.count) #平均取ったものをself.calculateを送る

        else :
            print("\n-----------------\nCan't find unko\n----------------------")
            self.kaisuu += 1
            print(self.kaisuu)
            #self.search(self.search_num)
            self.mode = ms.theta
        print("count = {}".format(self.count))
    
    def search(self, num):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper") 
        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        target_pose = geometry_msgs.msg.Pose()
        if num == 0:
            #searchの姿勢
            search_x = 0.2
            search_y = 0
            search_z = 0.3 
            self.sub = 0

        elif num == 1:
            #searchの姿勢
            search_x = 0.2
            search_y = 0.15
            search_z = 0.3
            self.sub = 0
        
        elif num == 2:
            #searchの姿勢
            search_x = 0.2
            search_y = -0.15
            search_z = 0.3
            self.sub = 0

        target_pose.position.x = search_x
        target_pose.position.y = search_y
        target_pose.position.z = search_z
        q = quaternion_from_euler(-3.14, 0, 3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()
        
        self.x = search_x 
        self.y = search_y
        self.z = search_z 
         
        print("\nsleepstart in search")
        rospy.sleep(7)
        print("sleepend in search")
        if num == 0:
            self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
        elif num == 1:
            self.sub = rospy.Subscriber("bool1", Pose2D, self.callback)
        elif num == 2:
            self.sub = rospy.Subscriber("bool2", Pose2D, self.callback)

    def pick(self):
       
        rad_y = 18  
        rad_x = rad_y * 1.33
        self.tar_x = self.z * math.tan(math.radians(rad_x)) * self.tar_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        self.tar_y = self.z * math.tan(math.radians(rad_y)) * self.tar_y / 240 #座標変換(画像の座標からセンサから見た実際の座標へ)
        print("calculated\n res_x:{}".format(self.tar_x))
        print("res_y:{}".format(self.tar_y))

        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        
        target_pose = geometry_msgs.msg.Pose()
        
        #offset_x = 0.05            #カメラと手先のオフセット
        offset_y = 0.03            #カメラと手先のオフセット
        
        tar_x = self.tar_y + self.x                         #センサの位置にあった原点をマニピュレータの根元へ移動
        tar_y = -self.tar_x + self.y + offset_y             #センサの位置にあった原点をマニピュレータの根元へ移動
        print("----------------------------") 
        print("tar_x = {}".format(tar_x)) 
        print("tar_y = {}".format(tar_y))
        print("----------------------------") 
        #つかむ前の位置、姿勢 
        target_pose.position.x = tar_x
        target_pose.position.y = tar_y
        target_pose.position.z = 0.15
        q = quaternion_from_euler(math.pi, 0.0, math.pi/2.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go() 
       
        #つかむ位置、姿勢
        target_pose.position.z = 0.11
        arm.set_pose_target(target_pose)
        arm.go() 

        #ハンドを閉じる
        gripper.set_joint_value_target([0.2,0.2])
        gripper.go()
       
        self.go()

    def go(self): 
        #投げる1(振りかぶり)
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
  
        joint_values = [0.00, 1.10, 0.00, -0.00, -0.00, 0.00, 0.00]
        
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=2.0)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        
        rospy.sleep(0.5) #何秒後にグリッパーを開くか

        #投げる2(中間)
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
  
        joint_values = [0.00, 0.80, 0.00, -0.00, -0.00, 0.00, 0.00]
        
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=0.1)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))
        

        #投げる3(中間)
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]
  
        joint_values = [0.00, 0.30, 0.00, -0.50, -0.00, 0.00, -0.00]
        
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=0.1)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)

        #投げ終わり
        point = JointTrajectoryPoint()
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["crane_x7_shoulder_fixed_part_pan_joint","crane_x7_shoulder_revolute_part_tilt_joint","crane_x7_upper_arm_revolute_part_twist_joint","crane_x7_upper_arm_revolute_part_rotate_joint","crane_x7_lower_arm_fixed_part_joint","crane_x7_lower_arm_revolute_part_joint","crane_x7_wrist_joint"]

        joint_values = [0.00, -0.15, -0.00, -0.80, -0.00, -0.00, -0.00]
        #-1.60
        position = math.radians(50.0)
        effort  = 1.0
        self.gripper_goal.command.position = position
        self.gripper_goal.command.max_effort = effort
                
        for i, p in enumerate(joint_values):
            point.positions.append(p)
        
        point.time_from_start = rospy.Duration(secs=0.1)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        
        rospy.sleep(0.05) #何秒後にグリッパーを開くか
        
        self.gripper_client.send_goal(self.gripper_goal,feedback_cb=self.feedback)
        self._client.wait_for_result(timeout=rospy.Duration(100.0))

        print("exit")
        sys.exit(1)
           
    def feedback(self,msg):
        print("feedback callback")

if __name__ == "__main__":
    rospy.init_node("unko_pitching")
    unko_pitching = ArmJointTrajectoryExample()
    unko_pitching.search_num = 0
    while unko_pitching.search_num < 3:
        unko_pitching.search(unko_pitching.search_num)
        rospy.sleep(8.0)
        print("changepose \n\n")
        if unko_pitching.mode == 1:
            unko_pitching.pick()
            unko_pitching.go()
        else :
            unko_pitching.search_num += 1
