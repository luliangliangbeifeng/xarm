#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
import time

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from xarm_msgs.srv import GripperMove, GripperConfig

redBox    = [0.344, 0.281, 0.176]  # 0.71931 -0.694677  0.00369  0.001 
yellowBox = [0.344, 0.238, 0.176]
greenBox  = [0.344, 0.199, 0.176]

placeBox  = [0.377, -0.189, 0.197]

class PickAndPlaceDemo:
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
        self.arm.set_max_acceleration_scaling_factor(0.3)
        self.arm.set_max_velocity_scaling_factor(0.3)

        # Initialize gripper
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
        response = self.gripper_client(200)

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
        target_pose.pose.orientation.x = 0.71931
        target_pose.pose.orientation.y = -0.694677
        target_pose.pose.orientation.z = 0.00369
        target_pose.pose.orientation.w = 0.001
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def moveSingleJoint(self, index, value):
        group_variable_values = self.arm.get_current_joint_values()
        group_variable_values[index] = value
        self.arm.set_joint_value_target(group_variable_values)
        traj = self.arm.plan()
        self.arm.execute(traj)
    
    def pick(self, x, y):
        if self.moveTo(x, y, 0.2) == True:
            print "Pick Once"
            self.moveSingleJoint(3, 0.2)
            
            self.CloseGripper()
            
            rospy.sleep(1)
            self.moveSingleJoint(3, 0.3)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y): 
        if self.moveTo(x, y, 0.2) == True:
            print "Place Once"
            
            self.OpenGripper()

            rospy.sleep(1)
        else:
            print "Can not place"

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    time.sleep(2)

    rospy.init_node('pick_place_demo')
    rate = rospy.Rate(10)
    
    print "Pick and place demo start."
    demo = PickAndPlaceDemo()
    
    # Red
    demo.moveTo(redBox[0], redBox[1], 0.3)
    
    demo.moveTo(redBox[0], redBox[1], redBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(redBox[0], redBox[1], 0.3)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)

    demo.moveTo(placeBox[0], placeBox[1], 0.185)

    demo.OpenGripper()
    rospy.sleep(1)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)

    # Yellow
    demo.moveTo(yellowBox[0], yellowBox[1], 0.3)
    
    demo.moveTo(yellowBox[0], yellowBox[1], yellowBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(yellowBox[0], yellowBox[1], 0.3)
    rospy.sleep(1)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)

    demo.moveTo(placeBox[0], placeBox[1], 0.218)

    demo.OpenGripper()
    rospy.sleep(1)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)


    # Green
    demo.moveTo(greenBox[0], greenBox[1], 0.3)
    
    demo.moveTo(greenBox[0], greenBox[1], greenBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(yellowBox[0], yellowBox[1], 0.3)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)

    demo.moveTo(placeBox[0], placeBox[1], 0.248)

    demo.OpenGripper()
    rospy.sleep(1)

    demo.moveTo(placeBox[0], placeBox[1], 0.3)

    demo.moveToHome()   
    demo.shutdown()
    
