#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        left_arm = moveit_commander.MoveGroupCommander('left_arm')
        right_arm = moveit_commander.MoveGroupCommander('right_arm')
        left_hand = moveit_commander.MoveGroupCommander('left_hand')
        right_hand = moveit_commander.MoveGroupCommander('right_hand')
        
        # 设置机械臂和夹爪的允许误差值
        left_arm.set_goal_joint_tolerance(0.001)
        right_arm.set_goal_joint_tolerance(0.001)
        left_hand.set_goal_joint_tolerance(0.001)
        right_hand.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        left_arm.set_named_target('left_arm_home')
        left_arm.go()
        rospy.sleep(2)

        right_arm.set_named_target('right_arm_home')
        right_arm.go()
        rospy.sleep(2)

        left_hand.set_named_target('left_hand_home')
        left_hand.go()
        rospy.sleep(1)

        right_hand.set_named_target('right_hand_home')
        right_hand.go()
        rospy.sleep(1)
         
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        left_arm.set_named_target('left_arm_up')
        left_arm.go()
        rospy.sleep(2)

        right_arm.set_named_target('right_arm_up')
        right_arm.go()
        rospy.sleep(2)

        left_arm.set_named_target('left_arm_kungfu')
        left_arm.go()
        rospy.sleep(2)

        right_arm.set_named_target('right_arm_kungfu')
        right_arm.go()
        rospy.sleep(2)

        left_hand.set_named_target('left_hand_thanks')
        left_hand.go()
        rospy.sleep(1)

        right_hand.set_named_target('right_hand_thanks')
        right_hand.go()
        rospy.sleep(1)

        left_arm.set_named_target('left_arm_thanks')
        left_arm.go()
        rospy.sleep(2)

        right_arm.set_named_target('right_arm_thanks')
        right_arm.go()
        rospy.sleep(2)
        # 控制机械臂先回到初始化位置
        left_arm.set_named_target('left_arm_home')
        left_arm.go()
        rospy.sleep(2)

        left_hand.set_named_target('left_hand_home')
        left_hand.go()
        rospy.sleep(1)

        right_hand.set_named_target('right_hand_good')
        right_hand.go()
        rospy.sleep(1)

        right_arm.set_named_target('right_arm_good')
        right_arm.go()
        rospy.sleep(2)

        right_hand.set_named_target('right_hand_home')
        right_hand.go()
        rospy.sleep(1)

        right_arm.set_named_target('right_arm_home')
        right_arm.go()
        rospy.sleep(2)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
