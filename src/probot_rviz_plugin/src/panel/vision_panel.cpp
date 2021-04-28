#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

#include "probot_rviz_plugin/panel/vision_panel.h"

namespace probot_rviz_plugin
{

VisionPanel::VisionPanel(QWidget* parent) : rviz::Panel(parent)
{
  //////////////Global image topic////////////////////////
  QGroupBox* imageTopicGroup = new QGroupBox(tr("Image Topic"));
  QVBoxLayout* imageTopicLayout = new QVBoxLayout;
  QHBoxLayout* rawImageTopicLayout = new QHBoxLayout;
  QHBoxLayout* outImageTopicLayout = new QHBoxLayout;

  rawImageTopicEdit_ = new QLineEdit("/camera/color/image_raw");
  outImageTopicEdit_ = new QLineEdit("/object_detect");

  rawImageTopicLayout->addWidget(new QLabel(tr("Raw Image Topic : ")));
  rawImageTopicLayout->addWidget(rawImageTopicEdit_);

  outImageTopicLayout->addWidget(new QLabel(tr("Out Image Topic : ")));
  outImageTopicLayout->addWidget(outImageTopicEdit_);
  
  captureImagePostionXEdit_ = new QLineEdit();
  captureImagePostionYEdit_ = new QLineEdit();
  captureImagePostionZEdit_ = new QLineEdit();
  goToCaptureButton_        = new QPushButton(tr("Go"));
  connect(goToCaptureButton_, SIGNAL(clicked(bool)), this, SLOT(onGoToCaptureBtnClicked(bool)));

  QHBoxLayout* captureImagePostionLayout = new QHBoxLayout;
  captureImagePostionLayout->addWidget(new QLabel(tr("Capture Position : ")));
  captureImagePostionLayout->addWidget(new QLabel(tr("X-")));
  captureImagePostionLayout->addWidget(captureImagePostionXEdit_);
  captureImagePostionLayout->addWidget(new QLabel(tr("Y-")));
  captureImagePostionLayout->addWidget(captureImagePostionYEdit_);
  captureImagePostionLayout->addWidget(new QLabel(tr("Z-")));
  captureImagePostionLayout->addWidget(captureImagePostionZEdit_);
  captureImagePostionLayout->addWidget(goToCaptureButton_);

  imageTopicLayout->addLayout(rawImageTopicLayout);
  imageTopicLayout->addLayout(outImageTopicLayout);
  imageTopicLayout->addLayout(captureImagePostionLayout);

  imageTopicGroup->setLayout(imageTopicLayout);


  tabWidget_ = new QTabWidget();

  //////////////hand eye calibration tab page////////////////////////
  handEyeCalibrationWidget_ = new HandEyeCalibrationWidget(rawImageTopicEdit_);

  //////////////image process config tab page////////////////////////
  imageProcessConfigWidget_ = new ImageProcessConfigWidget(rawImageTopicEdit_, outImageTopicEdit_);

  //////////////pick and place tab page////////////////////////
  pickAndPlaceWidgetWidget_ = new PickAndPlaceWidget(outImageTopicEdit_);

  tabWidget_->addTab(handEyeCalibrationWidget_, QString(tr("Calibration")));
  tabWidget_->addTab(imageProcessConfigWidget_, QString(tr("Config")));
  tabWidget_->addTab(pickAndPlaceWidgetWidget_, QString(tr("PickAndPlace")));


  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(imageTopicGroup);
  layout->addWidget(tabWidget_);

  setLayout(layout);

  robotVisionInterface_ = RobotVisionInterfacePtr(new RobotVisionInterface(nh_));
}

VisionPanel::~VisionPanel()
{
}

void VisionPanel::onGoToCaptureBtnClicked(bool checked)
{
  ROS_INFO("Go to capture image position btn clicked.");

  if(captureImagePostionXEdit_->text().isEmpty() ||
     captureImagePostionYEdit_->text().isEmpty() ||
     captureImagePostionZEdit_->text().isEmpty() )
  {
    ROS_ERROR("Input capture image position please!");
    return;
  }

  robotVisionInterface_->goToCaptureImagePosition(captureImagePostionXEdit_->text().toFloat(), 
                                                  captureImagePostionYEdit_->text().toFloat(), 
                                                  captureImagePostionZEdit_->text().toFloat());
}

void VisionPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void VisionPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}


} // end namespace probot_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(probot_rviz_plugin::VisionPanel, rviz::Panel)
