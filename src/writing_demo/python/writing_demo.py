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
import argparse
import subprocess
import math
import numpy
import yaml
import os
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose, PoseStamped, Point
from copy import deepcopy
from nav_msgs.msg import Path
from std_msgs.msg import Empty

import logging
logger = logging.getLogger("write.")
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setLevel(logging.DEBUG)
formatter = logging.Formatter('[%(levelname)s] %(name)s -> %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)
packagePath = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
print(packagePath)
yamlPath = os.path.join(packagePath,"config/writing.yaml")
print(yamlPath)
exePath = os.path.join(packagePath,"svg_subsampler")
print(exePath)
picturePath = os.path.join(packagePath,"pictures")
print(picturePath)
with open(yamlPath,'rb') as f:
     data = yaml.load(f)
     height = data["height"]
     density = data["density"]
     density = int(density/10)
     number = data["number"]
     picture1 = data["picture1"]
     pos1_x = data["pos1_x"]
     pos1_y = data["pos1_y"]
     picture2 = data["picture2"]
     pos2_x = data["pos2_x"]
     pos2_y = data["pos2_y"]
     picture3 = data["picture3"]
     pos3_x = data["pos3_x"]
     pos3_y = data["pos3_y"]    
p1Path = os.path.join(picturePath,picture1)
print(p1Path)
p2Path = os.path.join(picturePath,picture2)
print(p2Path)
p3Path = os.path.join(picturePath,picture3)
print(p3Path)

def get_traj(svgfile,waypoints, pos_x, pos_y, height):
    SVG_SAMPLER = "svg_subsampler"
    YFLIP = 'no-yflip'
    SAMPLE_DENSITY = density
    SAMPLE_TYPE = "homogeneous"
    logger.info("Running " + SVG_SAMPLER + "...")
    logger.info("%s\n",svgfile)
    logger.info("%d\n",SAMPLE_DENSITY)
    logger.info("%s\n",SAMPLE_TYPE)
    logger.info("%s\n",YFLIP) 
 
    p = subprocess.Popen([exePath, svgfile, str(SAMPLE_DENSITY), SAMPLE_TYPE, YFLIP], stdout=subprocess.PIPE, stderr =    subprocess.PIPE)
    
    traj, errors= p.communicate()
    logger.info(errors)
    count = 0
    traj = traj.strip().split("\n")
    # first line of the output of svg2traj is the x,y origin of the path in meters,
    # relative to the SVG document origin

    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.0
    target_pose.pose.orientation.x = 1.0

    x_orig, y_orig = [float(x) for x in traj[0].split()]
    for l in traj[1:]:
        x, y, z = l.split()
        if(SVG_SAMPLER=="svg2traj"):
            x=x_orig + float(x);
            y=y_orig + float(y);
        elif("svg_subsampler" in SVG_SAMPLER):
            x=float(x)
            y=float(y)
            target_pose.pose.position.x = -y + pos_x
            target_pose.pose.position.y = -x + pos_y
            target_pose.pose.position.z = height
            target_pose.pose.orientation.x = 1.0
            wpose = deepcopy(target_pose.pose)
            waypoints.append(deepcopy(wpose))
            count = count + 1
	    logger.info("points%d:  x:%f   y:%f",count, x, y)
	yield Point(x, y, 0) # strange values on Z! better set it to 0

def movetowrite(waypoints):
        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数

        arm = MoveGroupCommander('xarm6')
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

def moveup():
	arm = MoveGroupCommander('xarm6')
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

def moveTo(x, y, z): 
    arm = MoveGroupCommander('xarm6')
    # Create pose data           
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'world'
    target_pose.header.stamp = rospy.Time.now()     
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation.x = 1
    
    # Set pick position 
    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose, 'link6')
    
    traj = arm.plan()

    if len(traj.joint_trajectory.points) == 0:
        return False

    arm.execute(traj)
    
    return True

class MoveItCircleDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_clrcle_demo', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('xarm6')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'world'
        arm.set_pose_reference_frame('world')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.0003)
        arm.set_goal_orientation_tolerance(0.01)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.1)
        arm.set_max_velocity_scaling_factor(0.1)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)
        
	for i in range(0,number):
           moveTo(0.45, -0.1-0.06*i, 0.12)

           if i == 0:
               x = pos1_x
	       y = pos1_y
               add = p1Path
           elif i == 1:
               x = pos2_x
	       y = pos2_y
               add = p2Path
           elif i == 2:
               x = pos3_x
	       y = pos3_y
               add = p3Path
            
           waypoints = []
           raw_traj = list(get_traj(add, waypoints, x, y, height))#图像路径
           movetowrite(waypoints)


        moveTo(0.36, -0.29, 0.25)
        arm.set_named_target('home')
        arm.go()
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)                            

if __name__ == "__main__":
    try:
        MoveItCircleDemo()
    except rospy.ROSInterruptException:
        pass
