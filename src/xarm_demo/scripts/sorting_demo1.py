#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import time

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from object_color_detector.srv import *
from xarm_msgs.srv import GripperMove, GripperConfig

redBox    = [0.40, -0.02, 0.217]  # 0.71931 -0.694677  0.00369  0.001 
blueBox   = [0.40, -0.02, 0.287]
greenBox  = [0.40, -0.02, 0.252]



placeBox  = [0.377, -0.189, 0.197]

class VisionSortingDemo:
    def __init__(self):
        # Initialize ros and moveit 
        moveit_commander.roscpp_initialize(sys.argv)
                
        # Initialize moveit commander
        self.arm = moveit_commander.MoveGroupCommander('xarm6')
                
        # Initialize arm effector link
        self.end_effector_link = self.arm.get_end_effector_link()
                        
        # Initialize reference frame
        self.reference_frame = 'world'
        self.arm.set_pose_reference_frame(self.reference_frame)
                
        # Initialize moveit parameters
        self.arm.allow_replanning(True)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)

        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.8)
        self.arm.set_max_velocity_scaling_factor(0.8)

        # Initialize gripper
        rospy.wait_for_service('/xarm/gripper_config')
        try:
            self.gripper_client = rospy.ServiceProxy('/xarm/gripper_config', GripperConfig)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        response = self.gripper_client(1500)

        rospy.wait_for_service('/xarm/gripper_move')
        try:
            self.gripper_client = rospy.ServiceProxy('/xarm/gripper_move', GripperMove)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        #initialize arm position to home
        self.arm.set_named_target('hold-up')
        self.arm.go()
        rospy.sleep(1)
        self.OpenGripper()
        rospy.sleep(1)

    def OpenGripper(self):
        response = self.gripper_client(500)

    def CloseGripper(self):
        response = self.gripper_client(260)
	
    def CloseGripper_cy(self):
        response = self.gripper_client(360)

    def moveToHome(self):
        self.arm.set_named_target('hold-up')
        self.arm.go()
        rospy.sleep(1)
        self.OpenGripper()

    def moveTo(self, x, y, z): 
        # Create pose data           
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = 1
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def pick(self, x, y, z):
        if self.moveTo(x, y, z+0.1) == True:
            print "Pick Once"
            
            if self.moveTo(x, y, z) == True:
                print "Pick Once"
                
                if self.moveTo(x, y, z-0.025) == True:
                    self.CloseGripper()
                else:
                    print "Can not pick"
                    return False
                
                rospy.sleep(0.1)
                self.moveTo(x, y, 0.35)
                
                return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y, z): 
        if self.moveTo(x, y, z) == True:  #(x, y, 0.22)
            print "Place Once"

            if self.moveTo(x, y, z-0.035) == True:  #(x, y, 0.185)
                self.OpenGripper()
            else:
                print "Can not pick"
                return False
              
            rospy.sleep(0.1)
            self.moveTo(x, y, z)
        else:
            print "Can not place"

	#圆柱体抓取，直径为4cm
    def pick_cy(self, x, y, z):
        if self.moveTo(x, y, z+0.1) == True:
            print "Pick Once"
            
            if self.moveTo(x, y, z) == True:
                print "Pick Once"
                
                if self.moveTo(x, y, z-0.025) == True:
                    self.CloseGripper_cy()
                else:
                    print "Can not pick"
                    return False
                
                rospy.sleep(0.1)
                self.moveTo(x, y, 0.35)
                
                return True
        else:
            print "Can not pick"
            return False
       

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":

    time.sleep(2)

    rospy.init_node('sorting_demo')
    rate = rospy.Rate(10)

    reg_x = rospy.get_param('/image/reg_x')
    reg_y = rospy.get_param('/image/reg_y')
    
    print "Vision sorting demo start."
    demo = VisionSortingDemo()

    
    while not rospy.is_shutdown():
        # Initialize position
        demo.moveTo(0.333572, 0, 0.35)
        
		#正方体123
		#正方体10
        if demo.pick(0.23,  -0.04, 0.21) == True:
            demo.place(0.04, 0.25, 0.224)
		#正方体11
        if demo.pick(0.33,  -0.11, 0.21) == True:
            demo.place(0.04, 0.25, 0.26)
		#正方体12
        if demo.pick(0.37,  0.11, 0.21) == True:
            demo.place(0.04, 0.25, 0.293)
        

		#长方体456
		#长方体13
        if demo.pick(0.41,  -0.11, 0.205) == True:
            demo.place(0.13, 0.25, 0.223)
		#长方体14
        if demo.pick(0.44,  0.065, 0.205) == True:
            demo.place(0.13, 0.25, 0.247)
		#长方体15
        if demo.pick(0.26,  0.06, 0.205) == True:
            demo.place(0.13, 0.25, 0.266)
		
        
		#圆柱789
		#圆柱7
        if demo.pick_cy(0.4,  -0.03, 0.21) == True:
            demo.place(0.22, 0.25, 0.232)
		#圆柱8
        if demo.pick_cy(0.31,  -0.03, 0.21) == True:
            demo.place(0.22, 0.25, 0.277)
		#圆柱9
        if demo.pick_cy(0.35,  0.03, 0.21) == True:
            demo.place(0.22, 0.25, 0.309)
       
               
        rate.sleep()
        
        while not rospy.is_shutdown():
            demo.moveTo(0.333572, 0, 0.35)
            time.sleep(1)
        demo.moveToHome()   
        demo.shutdown() 
        
    demo.moveToHome()   
    demo.shutdown()            



