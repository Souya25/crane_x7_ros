#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

def main():
    rospy.init_node("arm_move")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(1)
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


    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

    # 左
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.0541737954085
    target_pose.position.y = 0.165643542357
    target_pose.position.z = 0.129005931973
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = -0.493243329595
    target_pose.orientation.y = 0.725260234009
    target_pose.orientation.z = 0.292664223979
    target_pose.orientation.w = 0.380862524776
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    rospy.sleep(2.0)

    arm.set_named_target("home")
    arm.go()
    #gripper.set_joint_value_target([0.7, 0.7])
    #gripper.go()

    # 左中
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =  0.222562384824
    target_pose.position.y =  0.153946645793
    target_pose.position.z =  0.145367063089
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = -0.37012010179
    target_pose.orientation.y =  0.672156907829
    target_pose.orientation.z = -0.578379233916
    target_pose.orientation.w = 0.276936207967
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    rospy.sleep(2.0)

    # ハンドを閉じる
   # gripper.set_joint_value_target([0.05, 0.05])
   # gripper.go()
    arm.set_named_target("home")
    arm.go()
    # 右中
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.171030885261
    target_pose.position.y = -0.173626113727
    target_pose.position.z = 0.162637874467
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = 0.167282244215
    target_pose.orientation.y = 0.86290183543
    target_pose.orientation.z = 0.216959851386
    target_pose.orientation.w = 0.424671044537
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()	

    rospy.sleep(2.0)

    # 実行
    arm.set_named_target("home")
    arm.go()
    # 右
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x =   0.0466286918116
    target_pose.position.y =  -0.129318720394
    target_pose.position.z =  0.156499396902
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = 0.505359038888
    target_pose.orientation.y = 0.710421862758
    target_pose.orientation.z = -0.308887185991
    target_pose.orientation.w = 0.380133825198
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    rospy.sleep(2.0)

    arm.set_named_target("home")
    arm.go()
    print("done")
if __name__ == '__main__':

    main()
   
  