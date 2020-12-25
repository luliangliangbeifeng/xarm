#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from copy import deepcopy

import math
import numpy

class MotionDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('motion_demo')
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('xarm6')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.3)
        arm.set_max_velocity_scaling_factor(0.3)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
               
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        position1_up = [0.6594585136028958, -0.3554481894298158, -0.21982017619867814, -2.607007985556644e-06, 0.5752497301969385, 0.6594537744441299]
        position1_down = [0.6595044295824808, 0.11466670433689617, -0.23117010290374929, -3.437144662110028e-05, 0.11656486718517368, 0.6594440876866068]
        position2_up = [-0.9055064264307319, -0.1317772311201235, -0.500498536398391, 2.7664533430529252e-05, 0.6323085401181505, -0.9056311636988187]
        position2_down = [-0.9055632033143053, 0.21458975263445956, -0.49008161922143584, -7.627886933981161e-06, 0.27547019005519396, -0.9056579060760584]

        # 设置机器臂当前的状态作为运动初始状态        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position1_down)           
        arm.go()
        
        arm.set_joint_value_target(position1_up)           
        arm.go()
        arm.set_joint_value_target(position2_up)           
        arm.go()
        arm.set_joint_value_target(position2_down)           
        arm.go()

        arm.set_joint_value_target(position2_up)           
        arm.go()

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = arm.get_current_pose(end_effector_link).pose

        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        # waypoints.append(start_pose)
            
        # 设置路点数据，并加入路点列表
        wpose = deepcopy(start_pose)
        wpose.position.z += 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.2
        waypoints.append(deepcopy(wpose))
        
        wpose.position.y += 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y -= 0.2
        waypoints.append(deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.001,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  


        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.35
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.20
        target_pose.pose.orientation.x = 1.0
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()

        # 初始化路点列表
        waypoints = []
                
        # 将圆弧上的路径点加入列表
        # waypoints.append(target_pose.pose)

        centerA = target_pose.pose.position.y
        centerB = target_pose.pose.position.x
        radius = 0.1

        for th in numpy.arange(0, 6.28, 0.02):
            target_pose.pose.position.y = centerA + radius * math.cos(th)
            target_pose.pose.position.x = centerB + radius * math.sin(th)
            wpose = deepcopy(target_pose.pose)
            waypoints.append(deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点，完成圆弧轨迹
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                     
        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MotionDemo()
    
