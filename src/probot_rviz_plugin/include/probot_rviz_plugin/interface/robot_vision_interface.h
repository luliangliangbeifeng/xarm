#ifndef PROBOT_ROBOT_VISION_INTERFACE_H
#define PROBOT_ROBOT_VISION_INTERFACE_H

#include <memory>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <xarm_msgs/GripperConfig.h>
#include <xarm_msgs/GripperMove.h>

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
                               std::vector<float>& pixelPositionY, std::vector<float>& calibrationResultX, std::vector<float>& calibrationResultY, std::string calibrationYaml);

  bool setRoiParameters(unsigned int roiX, unsigned int roiY, unsigned int roiHeight, unsigned int roiWidth);
  bool saveParameters(std::string yamlname);
  bool setHsvParameters(const std::vector<int>& imageMinHsv, const std::vector<int>& imageMaxHsv);

  bool objectRecognitionProcess(std::vector<unsigned int>& pixelPosition, std::vector<float>& objectPosition);
  bool pickAndPlaceProcess(std::vector<float>& pickPosition, std::vector<float>& placePosition);

  bool goToCaptureImagePosition(float posX, float posY, float posZ);

protected:
  
private:
  ros::NodeHandle &nh_; // The ROS node handle.
  int hsvmin[3];
  int hsvmax[3];
  int roivalue[4];

};

typedef std::shared_ptr<RobotVisionInterface> RobotVisionInterfacePtr;
typedef std::shared_ptr<const RobotVisionInterface> RobotVisionInterfaceConstPtr;

} // end namespace probot_rviz_plugin

#endif // PROBOT_ROBOT_VISION_INTERFACE_H
