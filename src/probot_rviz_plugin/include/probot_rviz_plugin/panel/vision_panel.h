#ifndef PROBOT_VISION_PANEL_H
#define PROBOT_VISION_PANEL_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#endif

#include "probot_rviz_plugin/widget/hand_eye_calibration_widget.h"
#include "probot_rviz_plugin/widget/image_process_config_widget.h"
#include "probot_rviz_plugin/widget/pick_and_place_widget.h"

#include "probot_rviz_plugin/interface/robot_vision_interface.h"

namespace probot_rviz_plugin
{

class VisionPanel: public rviz::Panel
{
Q_OBJECT
public:
  VisionPanel(QWidget* parent = 0);
  virtual ~VisionPanel();

  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected Q_SLOTS:
  void onGoToCaptureBtnClicked(bool checked);

protected:

private:
  ros::NodeHandle nh_; // The ROS node handle.

  QLineEdit  *rawImageTopicEdit_;
  QLineEdit  *outImageTopicEdit_;

  QLineEdit  *captureImagePostionXEdit_;
  QLineEdit  *captureImagePostionYEdit_;
  QLineEdit  *captureImagePostionZEdit_;

  QPushButton* goToCaptureButton_;  

  QTabWidget* tabWidget_;
  HandEyeCalibrationWidget* handEyeCalibrationWidget_;
  ImageProcessConfigWidget* imageProcessConfigWidget_;
  PickAndPlaceWidget* pickAndPlaceWidgetWidget_;

  RobotVisionInterfacePtr robotVisionInterface_;
};

} // end namespace probot_rviz_plugin

#endif // PROBOT_VISION_PANEL_H
