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
        self.num = 0
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        #self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
         
    global sum_x
    sum_x = 0
    global sum_y
    sum_y = 0
    global count
    count = 0
    global x
    x = 0
    global y
    y = 0
    global z
    z = 0
           
    def callback(self, ms):
        global mode
        global count
        
        if ms.theta == 1: #ms.theta == 1 no notoki kiiro naiyo 
                
            if count < 200:
                global sum_x
                sum_x += ms.x #軸変換(センサのy軸をマニピュレータのx軸に合わせる)
                global sum_y
                sum_y += ms.y #軸変換(センサのx軸をマニピュレータのy軸に合わせる)
                count += 1
            else:
                print("ave_x :{}".format(sum_x/count))
                print("ave_y :{}".format(sum_y/count))
                self.calculate(sum_x/count,sum_y/count) #平均取ったものをself.calculateを送る
        else :
            print("-----------------\nCan't find unko\n----------------------")
            self.num +=1
            print(self.num)
            self.search(self.num)

        print("count = {}".format(count))
    
    def calculate( self, ave_x, ave_y):
        
        print("ave_x:{}".format(ave_x))
        print("ave_y:{}".format(ave_y))
        rad_y = 18  
        rad_x = rad_y * 1.33
        res_x = z * math.tan(math.radians(rad_x)) * ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        #res_x = z * math.tan(math.radians(34.7)) * ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        res_y = z * math.tan(math.radians(rad_y)) * ave_y / 240 #座標変換(画像の座標からセンサから見た実際の座標へ)
        print("calculated\n res_x:{}".format(res_x))
        print("res_y:{}".format(res_y))

        self.pick( res_x, res_y)
                      
    def search(self, num):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)
         
        print("Group names:")
        print(robot.get_group_names())

        print("Current state:")
        print(robot.get_current_state())

        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose
        print("Arm initial pose:")
        print(arm_initial_pose)

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("home")
        arm.go()
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()
          
        target_pose = geometry_msgs.msg.Pose()
        
        #つかむ前の姿勢
        search_x = 0.2
        search_y = 0
        search_z = 0.3
        global x
        x = search_x 
        global y
        y = search_y
        global z
        z = search_z 
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
        
        print("go end") 
        rospy.sleep(2.0)
        print("sleep end")
        global mode
        mode = 0 
        self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
    
           
    
    def pick(self, res_x, res_y):
       
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        
        target_pose = geometry_msgs.msg.Pose()
        
        offset_x = 0.05            #カメラと手先のオフセット
        offset_y = 0.03            #カメラと手先のオフセット
        
        tar_x = res_y + x + offset_x                        #センサの位置にあった原点をマニピュレータの根元へ移動
        tar_y = -res_x + y + offset_y             #センサの位置にあった原点をマニピュレータの根元へ移動
        
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
       
        #homeへ 
        arm.set_named_target("home")
        arm.go()
        mode = 1
        self.go(mode)

    def go(self, mode):
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
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample() 
    arm_joint_trajectory_example.search(0)
    rospy.spin()
