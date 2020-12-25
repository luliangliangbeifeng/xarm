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
from xarm_msgs.srv import GripperMove, GripperConfig, SetDigitalIO

redBox    = [0.40, -0.02, 0.18]  # 0.71931 -0.694677  0.00369  0.001 
blueBox   = [0.30, -0.02, 0.18]
greenBox  = [0.50, -0.02, 0.18]

pickRedBox    = [0.3138, 0.1235, 0.172]  
pickGreenBox  = [0.3528, 0.1235, 0.172]
pickYellowBox = [0.3929, 0.1235, 0.172]

placeBox  = [0.377, -0.189, 0.197]

vonveyer_belt_up    = [0.027, 0.398, 0.215]
vonveyer_belt_down  = [0.384, 0.398, 0.215]

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
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_max_velocity_scaling_factor(0.2)

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
        response = self.gripper_client(600)

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
        target_pose.pose.orientation.x = 1
        
        # Set pick position 
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        traj = self.arm.plan()

        if len(traj.joint_trajectory.points) == 0:
            return False

        self.arm.execute(traj)
        
        return True
    
    def pick(self, x, y):
        if self.moveTo(x, y, 0.25) == True:
            print "Pick Once"
            
            if self.moveTo(x, y, 0.215) == True:
                self.CloseGripper()
            else:
                print "Can not pick"
                return False
            
            rospy.sleep(1)
            self.moveTo(x, y, 0.25)
            
            return True
        else:
            print "Can not pick"
            return False
        
    def place(self, x, y): 
        if self.moveTo(x, y, 0.25) == True:
            print "Place Once"

            if self.moveTo(x, y, 0.18) == True:
                self.OpenGripper()
            else:
                print "Can not pick"
                return False
              
            rospy.sleep(1)
            self.moveTo(x, y, 0.25)
        else:
            print "Can not place"

    def shutdown(self):
        # Exit
        print "The demo is shutting down."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":

    time.sleep(3)

    rospy.init_node('vision_sorting_demo')
    rate = rospy.Rate(10)

    reg_x = rospy.get_param('/image/reg_x')
    reg_y = rospy.get_param('/image/reg_y')
    
    print "Vision sorting demo start."
    demo = VisionSortingDemo()

#############################################Red#################################
    # Pick
    demo.moveTo(pickRedBox[0], pickRedBox[1], 0.3)
    demo.moveTo(pickRedBox[0], pickRedBox[1], pickRedBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(pickRedBox[0], pickRedBox[1], 0.3)

    # Up
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], vonveyer_belt_up[2])

    demo.OpenGripper()
    rospy.sleep(2)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)

    # begin
    rospy.wait_for_service('/xarm/set_controller_dout')
    try:
        io_client = rospy.ServiceProxy('/xarm/set_controller_dout', SetDigitalIO)
    except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    response = io_client(4, 0)

    rospy.sleep(10)

    response = io_client(4, 1)

#############################################Green#################################
    # Pick
    demo.moveTo(pickGreenBox[0], pickGreenBox[1], 0.3)
    demo.moveTo(pickGreenBox[0], pickGreenBox[1], pickGreenBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(pickGreenBox[0], pickGreenBox[1], 0.3)

    # Up
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], vonveyer_belt_up[2])

    demo.OpenGripper()
    rospy.sleep(2)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)


    # Initialize position
    demo.moveTo(0.3165, 0.3046, 0.33)

    # Get target
    rospy.wait_for_service('/object_detect')
    try:
        detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
        response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if response.result is not DetectObjectSrvResponse.SUCCESS:
        rospy.loginfo("No objects detected, waiting detecting...")
        rospy.sleep(10)
    
    rospy.loginfo("Get object position, Start pick and place.")

    # Pick and place bject
    if len(response.redObjList):
        x_value = response.redObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.redObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(redBox[0], redBox[1])
    elif len(response.greenObjList):
        x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(greenBox[0], greenBox[1])
    elif len(response.blueObjList):
        x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blueBox[0], blueBox[1])
    elif len(response.blackObjList):
        x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blackBox[0], blackBox[1])


    # begin
    rospy.wait_for_service('/xarm/set_controller_dout')
    try:
        io_client = rospy.ServiceProxy('/xarm/set_controller_dout', SetDigitalIO)
    except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    response = io_client(4, 0)

    rospy.sleep(10)

    response = io_client(4, 1)

#############################################Yellow#################################
    # Pick
    demo.moveTo(pickYellowBox[0], pickYellowBox[1], 0.3)
    demo.moveTo(pickYellowBox[0], pickYellowBox[1], pickYellowBox[2])

    demo.CloseGripper()
    rospy.sleep(2)

    demo.moveTo(pickYellowBox[0], pickYellowBox[1], 0.3)

    # Up
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], vonveyer_belt_up[2])

    demo.OpenGripper()
    rospy.sleep(2)
    demo.moveTo(vonveyer_belt_up[0], vonveyer_belt_up[1], 0.3)


    # Initialize position
    demo.moveTo(0.3165, 0.3046, 0.33)

    # Get target
    rospy.wait_for_service('/object_detect')
    try:
        detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
        response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if response.result is not DetectObjectSrvResponse.SUCCESS:
        rospy.loginfo("No objects detected, waiting detecting...")
        rate.sleep()
        rospy.sleep(10)
    
    rospy.loginfo("Get object position, Start pick and place.")

    # Pick and place bject
    if len(response.redObjList):
        x_value = response.redObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.redObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(redBox[0], redBox[1])
    elif len(response.greenObjList):
        x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(greenBox[0], greenBox[1])
    elif len(response.blueObjList):
        x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blueBox[0], blueBox[1])
    elif len(response.blackObjList):
        x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blackBox[0], blackBox[1])


    # begin
    rospy.wait_for_service('/xarm/set_controller_dout')
    try:
        io_client = rospy.ServiceProxy('/xarm/set_controller_dout', SetDigitalIO)
    except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    response = io_client(4, 0)

    rospy.sleep(10)

    response = io_client(4, 1)

    # Initialize position
    demo.moveTo(0.3165, 0.3046, 0.33)

    # Get target
    rospy.wait_for_service('/object_detect')
    try:
        detect_object_service = rospy.ServiceProxy('/object_detect', DetectObjectSrv)
        response = detect_object_service(DetectObjectSrvRequest.ALL_OBJECT) 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    if response.result is not DetectObjectSrvResponse.SUCCESS:
        rospy.loginfo("No objects detected, waiting detecting...")
        rate.sleep()
        rospy.sleep(10)
    
    rospy.loginfo("Get object position, Start pick and place.")

    # Pick and place bject
    if len(response.redObjList):
        x_value = response.redObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.redObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(redBox[0], redBox[1])
    elif len(response.greenObjList):
        x_value = response.greenObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.greenObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(greenBox[0], greenBox[1])
    elif len(response.blueObjList):
        x_value = response.blueObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blueObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blueBox[0], blueBox[1])
    elif len(response.blackObjList):
        x_value = response.blackObjList[0].position.y * reg_x[0] + reg_x[1]
        y_value = response.blackObjList[0].position.x * reg_y[0] + reg_y[1]
        print "Pick Position: %f, %f"%(x_value, y_value)
        if demo.pick(x_value,  y_value) == True:
            demo.place(blackBox[0], blackBox[1])

    demo.moveToHome()   
    demo.shutdown()
