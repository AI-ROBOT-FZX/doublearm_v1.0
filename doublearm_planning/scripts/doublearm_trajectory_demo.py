#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('doublearm_trajectory_demo')
        
        # 是否需要回到初始化的位置
        reset = rospy.get_param('~reset', False)
        
        # 机械臂中joint的命名
        doublearm_joints = ['left_joint2',
                      'left_joint3', 
                      'left_joint4',
                      'left_joint5',
                      'left_joint6',
		              'left_palm_joint',
                      'left_thumb_joint',
                      'left_index_finger_joint',
                      'left_middle_finger_joint',
                      'left_third_finger_joint',
                      'left_little_finger_joint',
		              'right_joint2',
                      'right_joint3', 
                      'right_joint4',
                      'right_joint5',
                      'right_joint6',
		              'right_palm_joint',
                      'right_thumb_joint',
                      'right_index_finger_joint',
                      'right_middle_finger_joint',
                      'right_third_finger_joint',
                      'right_little_finger_joint']
        
        if reset:
            # 如果需要回到初始化位置，需要将目标位置设置为初始化位置的六轴角度
            doublearm_goal  = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        else:
            # 如果不需要回初始化位置，则设置目标位置的六轴角度
            doublearm_goal  = [1,0.25,0.48,0.8,0.4,0.25,0.25,0.25,0.25,0.25,0.25,1,0.25,0.48,0.8,0.4,0.25,0.25,0.25,0.25,0.25,0.25]
    
        # 连接机械臂轨迹规划的trajectory action server
        rospy.loginfo('Waiting for doublearm trajectory controller...')       
        doublearm_client = actionlib.SimpleActionClient('doublearm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        doublearm_client.wait_for_server()        
        rospy.loginfo('...connected.')  
    
        # 使用设置的目标位置创建一条轨迹数据
        doublearm_trajectory = JointTrajectory()
        doublearm_trajectory.joint_names = doublearm_joints
        doublearm_trajectory.points.append(JointTrajectoryPoint())
        doublearm_trajectory.points[0].positions = doublearm_goal
        doublearm_trajectory.points[0].velocities = [0.0 for i in doublearm_joints]
        doublearm_trajectory.points[0].accelerations = [0.0 for i in doublearm_joints]
        doublearm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        rospy.loginfo('Moving the doublearm to goal position...')
        
        # 创建一个轨迹目标的空对象
        doublearm_goal = FollowJointTrajectoryGoal()
        
        # 将之前创建好的轨迹数据加入轨迹目标对象中
        doublearm_goal.trajectory = doublearm_trajectory
        
        # 设置执行时间的允许误差值
        doublearm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # 将轨迹目标发送到action server进行处理，实现机械臂的运动控制
        doublearm_client.send_goal(doublearm_goal)

        # 等待机械臂运动结束
        doublearm_client.wait_for_result(rospy.Duration(5.0))
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
