#ifndef PROBOT_ROBOT_VISION_INTERFACE_H
#define PROBOT_ROBOT_VISION_INTERFACE_H

#include <memory>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include  "yaml-cpp/yaml.h"
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>

namespace probot_rviz_plugin
{

class RobotVisionInterface
{
public:
  RobotVisionInterface(ros::NodeHandle& nh);
  ~RobotVisionInterface();

  bool patternRecognitionProcess(std::string& imageTopicName, std::vector<float>& pixelPositionX, std::vector<float>& pixelPositionY);
  bool imageCalibrationProcess(std::vector<float>& robotPositionX, std::vector<float>& robotPositionY, std::vector<float>& pixelPositionX, 
                               std::vector<float>& pixelPositionY, std::vector<float>& calibrationResultX, std::vector<float>& calibrationResultY);

protected:
  
private:
  ros::NodeHandle &nh_; // The ROS node handle.

};

typedef std::shared_ptr<RobotVisionInterface> RobotVisionInterfacePtr;
typedef std::shared_ptr<const RobotVisionInterface> RobotVisionInterfaceConstPtr;

} // end namespace probot_rviz_plugin

#endif // PROBOT_ROBOT_VISION_INTERFACE_H
