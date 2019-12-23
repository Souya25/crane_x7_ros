#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from subprocess import call
from subprocess import Popen
from subprocess import PIPE
flag_demo = 0
key = 0
def kill_node(nodename): 
	p2=Popen(['rosnode','list'],stdout=PIPE) 
	p2.wait() 
	nodelist=p2.communicate() 
	nd=nodelist[0] 
	nd=nd.split("\n") 
	for i in range(len(nd)): 
		tmp=nd[i] 
		ind=tmp.find(nodename) 
		if ind==1: 
			call(['rosnode','kill',nd[i]]) 
			break 

def callback(msg):
    kill_node('arm_move')
    rospy.sleep(12.0)
    bool_c = msg.data
    print (bool_c)
    body_up()
    rospy.sleep(10000.0)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("bool", String, callback)
    rospy.spin()
    


def body_up():
     # 取りに行く

    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

  

    arm.set_named_target("home")
    arm.go()

      # ハンドを閉じる
    gripper.set_joint_value_target([0.8, 0.8])
    gripper.go()

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.205045831868
    target_pose.position.y = -0.0194597655966
    target_pose.position.z = 0.309807278119
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = 0.474871942335
    target_pose.orientation.y = 0.424353826113
    target_pose.orientation.z = 0.514882237833
    target_pose.orientation.w = 0.573861263557
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行take

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =  0.269119370689
    target_pose.position.y = -0.0439712214052
    target_pose.position.z = 0.138551614587
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = 0.717972912926
    target_pose.orientation.y = 0.671395173952
    target_pose.orientation.z = 0.10624984559
    target_pose.orientation.w =  0.149847879569
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行take

    # ハンドを閉じる
    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

    arm.set_named_target("home")
    arm.go()

    # 物体を持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = -0.0372058227489
    target_pose.position.y = -0.246006903877
    target_pose.position.z = 0.169424000349
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x =  -0.475279946015
    target_pose.orientation.y = 0.431250538961
    target_pose.orientation.z = -0.753061012631
    target_pose.orientation.w = 0.145020884072
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()
    
    rospy.sleep(2.0)

       # ハンドを閉じる
    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()
    
    arm.set_named_target("home")
    arm.go()
    print("done")

if __name__ == '__main__':

    listener()