#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        left_arm = moveit_commander.MoveGroupCommander('left_arm')
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
                
        # 获取终端link的名称
        left_end_effector_link = left_arm.get_end_effector_link()
        right_end_effector_link = right_arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        left_arm.set_pose_reference_frame(reference_frame)
        right_arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        left_arm.allow_replanning(True)
        right_arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        left_arm.set_goal_position_tolerance(0.01)
        left_arm.set_goal_orientation_tolerance(0.05)
        right_arm.set_goal_position_tolerance(0.01)
        right_arm.set_goal_orientation_tolerance(0.05)
        
        # 控制机械臂先回到初始化位置
        left_arm.set_named_target('left_arm_home')
        left_arm.go()
        rospy.sleep(2)
        right_arm.set_named_target('right_arm_home')
        right_arm.go()
        rospy.sleep(2)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        left_target_pose = PoseStamped()
        left_target_pose.header.frame_id = reference_frame
        left_target_pose.header.stamp = rospy.Time.now()     
        left_target_pose.pose.position.x = 0
        left_target_pose.pose.position.y = 0
        left_target_pose.pose.position.z = 0
        left_target_pose.pose.orientation.x = 0
        left_target_pose.pose.orientation.y = 0
        left_target_pose.pose.orientation.z = 0
        left_target_pose.pose.orientation.w = 0

        right_target_pose = PoseStamped()
        right_target_pose.header.frame_id = reference_frame
        right_target_pose.header.stamp = rospy.Time.now()     
        right_target_pose.pose.position.x = 0
        right_target_pose.pose.position.y = 0
        right_target_pose.pose.position.z = 0
        right_target_pose.pose.orientation.x = 0
        right_target_pose.pose.orientation.y = 0
        right_target_pose.pose.orientation.z = 0
        right_target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        left_arm.set_start_state_to_current_state()
        right_arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        left_arm.set_pose_target(left_target_pose, left_end_effector_link)
        right_arm.set_pose_target(right_target_pose, right_end_effector_link)
        
        # 规划运动路径
        left_traj = left_arm.plan()
        right_traj = right_arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        left_arm.execute(left_traj)
        rospy.sleep(1)
        right_arm.execute(right_traj)
        rospy.sleep(1)
         
        # 控制机械臂终端向右移动5cm
        left_arm.shift_pose_target(1, 0, left_end_effector_link)
        left_arm.go()
        rospy.sleep(1)
        right_arm.shift_pose_target(1, 0, right_end_effector_link)
        right_arm.go()
        rospy.sleep(1)
  
        # 控制机械臂终端反向旋转90度
        left_arm.shift_pose_target(3, 0, left_end_effector_link)
        left_arm.go()
        rospy.sleep(1)
        right_arm.shift_pose_target(3, 0, right_end_effector_link)
        right_arm.go()
        rospy.sleep(1)
           
        # 控制机械臂回到初始化位置
        left_arm.set_named_target('left_arm_home')
        left_arm.go()
        rospy.sleep(1)
        right_arm.set_named_target('right_arm_home')
        right_arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
