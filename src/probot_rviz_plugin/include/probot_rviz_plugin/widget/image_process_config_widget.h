#ifndef PROBOT_IMAGE_PROCESS_CONFIG_WIDGET_H
#define PROBOT_IMAGE_PROCESS_CONFIG_WIDGET_H

#include <ros/ros.h>

#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>

#include "probot_rviz_plugin/interface/robot_vision_interface.h"
#include <string>

namespace probot_rviz_plugin
{

class ImageProcessConfigWidget: public QWidget
{
Q_OBJECT
public:
  ImageProcessConfigWidget(QLineEdit *rawImageTopicEdit, QLineEdit *outImageTopicEdit, QWidget* parent = 0);

protected Q_SLOTS:
  void onRoiSetBtnClicked(bool checked = false);
  void onParamSaveBtnClicked(bool checked = false);

  void hmaxValueSliderValueChanged(int value);
  void hminValueSliderValueChanged(int value);
  void smaxValueSliderValueChanged(int value);
  void sminValueSliderValueChanged(int value);
  void vmaxValueSliderValueChanged(int value);
  void vminValueSliderValueChanged(int value);

private:
  ros::NodeHandle nh_; // The ROS node handle.

  QLineEdit  *rawImageTopicEdit_;
  QLineEdit  *outImageTopicEdit_;

  QLineEdit  *roiXEdit_;
  QLineEdit  *roiYEdit_;
  QLineEdit  *roiHeightEdit_;
  QLineEdit  *roiWidthEdit_;

  QPushButton* roiSetBtn_;

  QLineEdit  *directoryEdit_;
  QPushButton* paramSaveBtn_;

  QSlider *hmaxValueSlider_;
  QSlider *hminValueSlider_;
  QSlider *smaxValueSlider_;
  QSlider *sminValueSlider_;
  QSlider *vmaxValueSlider_;
  QSlider *vminValueSlider_;

  QLabel *hmaxValueLabel_;
  QLabel *hminValueLabel_;
  QLabel *smaxValueLabel_;
  QLabel *sminValueLabel_;
  QLabel *vmaxValueLabel_;
  QLabel *vminValueLabel_;

  std::vector<int> imageMinHsv_;
  std::vector<int> imageMaxHsv_;

  RobotVisionInterfacePtr robotVisionInterface_;
};

} // end namespace probot_rviz_plugin


#endif // PROBOT_IMAGE_PROCESS_CONFIG_WIDGET_H
