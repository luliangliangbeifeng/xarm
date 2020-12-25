/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>
#include <xarm_planner/exec_plan.h>
#include <stdlib.h>
#include <vector>

bool request_plan(ros::ServiceClient& client, xarm_planner::joint_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service joint_plan");
		return false;
	}
}

bool request_exec(ros::ServiceClient& client, xarm_planner::exec_plan& srv)
{
	if(client.call(srv))
	{
		return srv.response.success;
	}
	else
	{
		ROS_ERROR("Failed to call service exec_plan");
		return false;
	}
}


int main(int argc, char** argv)
{	
	std::vector<double> tar_joint1 = {-1.0, -0.75, 0.0, -0.5, 0.0, 0.3, 0.0};
	std::vector<double> tar_joint2 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> tar_joint3 = {1.0, -0.75, 0.0, -0.5, 0.0, -0.3, 0.0};

	ros::init(argc, argv, "xarm_simple_planner_client");
	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<xarm_planner::joint_plan>("xarm_joint_plan");
	ros::ServiceClient client_exec = nh.serviceClient<xarm_planner::exec_plan>("xarm_exec_plan");

	ros::Publisher exec_pub = nh.advertise<std_msgs::Bool>("xarm_planner_exec", 10);
	std_msgs::Bool msg;
	xarm_planner::joint_plan srv;
	xarm_planner::exec_plan srv_exec;


	srv.request.target = tar_joint1;
	if(request_plan(client, srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	// ros::Duration(4.0).sleep(); // Wait for last execution to finish

	srv.request.target = tar_joint2;
	if(request_plan(client, srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	// ros::Duration(4.0).sleep(); // Wait for last execution to finish

	srv.request.target = tar_joint3;
	if(request_plan(client, srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	// ros::Duration(4.0).sleep(); // Wait for last execution to finish

	srv.request.target = tar_joint2;
	if(request_plan(client, srv))
	{
		ROS_INFO("Plan SUCCESS! Executing... ");
		// msg.data = true;
		// ros::Duration(1.0).sleep();
		// exec_pub.publish(msg);
		srv_exec.request.exec = true;
		request_exec(client_exec, srv_exec);
	}

	return 0;

}

