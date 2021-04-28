#ifndef PROBOT_PICK_AND_PLACE_WIDGET_H
#define PROBOT_PICK_AND_PLACE_WIDGET_H

#include <ros/ros.h>

#include <QPushButton>
#include <QLineEdit>

#include "probot_rviz_plugin/interface/robot_vision_interface.h"

namespace probot_rviz_plugin
{

class PickAndPlaceWidget: public QWidget
{
Q_OBJECT
public:
  PickAndPlaceWidget(QLineEdit  *outImageTopicEdit, QWidget* parent = 0);

protected Q_SLOTS:
  void onObjectRecognitionBtnClicked(bool checked = false);
  void onPickAndPlaceBtnClicked(bool checked = false);
  
private:
  ros::NodeHandle nh_; // The ROS node handle.

  QLineEdit  *outImageTopicEdit_;

  QLineEdit  *pixelPositionXEdit_;
  QLineEdit  *pixelPositionYEdit_;
  QLineEdit  *pixelPositionZEdit_;

  QLineEdit  *objectPositionXEdit_;
  QLineEdit  *objectPositionYEdit_;
  QLineEdit  *objectPositionZEdit_;
  
  QLineEdit  *placePositionXEdit_;
  QLineEdit  *placePositionYEdit_;
  QLineEdit  *placePositionZEdit_;

  QPushButton* objectRecognitionBtn_;
  QPushButton* pickAndPlaceStartBtn_;

  RobotVisionInterfacePtr robotVisionInterface_;
};

} // end namespace probot_rviz_plugin


#endif // PROBOT_PICK_AND_PLACE_WIDGET_H
