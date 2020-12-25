#ifndef PROBOT_VISION_PANEL_H
#define PROBOT_VISION_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include <QPushButton>
#include <QComboBox>
#include <QLineEdit>

#include "probot_rviz_plugin/interface/robot_vision_interface.h"

namespace probot_rviz_plugin
{

#define MAX_CALIBRATION_POINT_COUNT  (9)

class VisionPanel: public rviz::Panel
{
Q_OBJECT
public:
  VisionPanel(QWidget* parent = 0);
  virtual ~VisionPanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void onPatternRecognitionBtnClicked(bool checked = false);
  void onImageCalibrationUpdateBtnClicked(bool checked = false);

protected:

private:
  QPushButton* patternRecognitionBtn_;
  QPushButton* imageCalibrationBtn_;

  QLineEdit  *imageTopicEdit_;
  QLineEdit  *robotPositionXEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *robotPositionYEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *pixelPositionXEdit_[MAX_CALIBRATION_POINT_COUNT];
  QLineEdit  *pixelPositionYEdit_[MAX_CALIBRATION_POINT_COUNT];

  QLineEdit  *calibrationResultXEdit_[3];
  QLineEdit  *calibrationResultYEdit_[3];

  ros::NodeHandle nh_; // The ROS node handle.

  RobotVisionInterfacePtr robotVisionInterface_;
};

} // end namespace probot_rviz_plugin

#endif // PROBOT_VISION_PANEL_H
