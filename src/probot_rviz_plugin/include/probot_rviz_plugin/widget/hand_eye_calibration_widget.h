#ifndef PROBOT_HAND_EYE_CALIBRATION_WIDGET_H
#define PROBOT_HAND_EYE_CALIBRATION_WIDGET_H

#include <ros/ros.h>

#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>

#include "probot_rviz_plugin/interface/robot_vision_interface.h"

namespace probot_rviz_plugin
{

#define MAX_CALIBRATION_POINT_COUNT  (9)

class HandEyeCalibrationWidget: public QWidget
{
Q_OBJECT
public:
  HandEyeCalibrationWidget(QLineEdit *rawImageTopicEdit_, QWidget* parent = 0);

protected Q_SLOTS:
  void onPatternRecognitionBtnClicked(bool checked = false);
  void onImageCalibrationUpdateBtnClicked(bool checked = false);

private:
  QPushButton* patternRecognitionBtn_;
  QPushButton* imageCalibrationBtn_;

  QLineEdit  *imageTopicEdit_;
  QLineEdit  *robotPositionXEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *robotPositionYEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *pixelPositionXEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *pixelPositionYEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *calibrationDirectoryEdit_;

  QLineEdit  *calibrationResultXEdit_[3];
  QLineEdit  *calibrationResultYEdit_[3];

  ros::NodeHandle nh_; // The ROS node handle.

  RobotVisionInterfacePtr robotVisionInterface_;
};

} // end namespace probot_rviz_plugin


#endif // PROBOT_HAND_EYE_CALIBRATION_WIDGET_H
