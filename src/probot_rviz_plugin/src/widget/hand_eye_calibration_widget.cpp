#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>

#include "probot_rviz_plugin/widget/hand_eye_calibration_widget.h"

namespace probot_rviz_plugin
{

HandEyeCalibrationWidget::HandEyeCalibrationWidget(QLineEdit *rawImageTopicEdit_, QWidget* parent)
  : imageTopicEdit_(rawImageTopicEdit_), QWidget(parent)
{
  //Calibration Group
  QGroupBox* calibrationGroup = new QGroupBox(tr("Vision Calibration"));
  QVBoxLayout* calibrationLayout = new QVBoxLayout;

  patternRecognitionBtn_ = new QPushButton(tr("Pattern Recognition"));
  patternRecognitionBtn_->setToolTip(tr("标定靶识别"));

  imageCalibrationBtn_ = new QPushButton(tr("Image Calibration"));
  imageCalibrationBtn_->setToolTip(tr("视觉标定"));

  calibrationLayout->addWidget(patternRecognitionBtn_);
  calibrationLayout->addWidget(imageCalibrationBtn_);

  QGroupBox* calibrationResultGroup = new QGroupBox(tr("Result"));
  QVBoxLayout* calibrationResultLayout = new QVBoxLayout;

  QHBoxLayout* calibrationDirectoryLayout = new QHBoxLayout;
  calibrationDirectoryEdit_ = new QLineEdit("/home/tianbot/xarm_ws/src/probot_rviz_plugin/config/calibration_config.yaml");

  calibrationDirectoryLayout->addWidget(new QLabel(tr("dir:")));
  calibrationDirectoryLayout->addWidget(calibrationDirectoryEdit_);

  calibrationResultLayout->addLayout(calibrationDirectoryLayout);

  for(unsigned int i=0; i<3; i++)
  {
    QHBoxLayout* calibrationResultLineLayout = new QHBoxLayout;
    calibrationResultXEdit_[i] = new QLineEdit();
    calibrationResultYEdit_[i] = new QLineEdit();

    calibrationResultXEdit_[i]->setEnabled(false);
    calibrationResultYEdit_[i]->setEnabled(false);

    calibrationResultLineLayout->addWidget(calibrationResultXEdit_[i]);
    calibrationResultLineLayout->addWidget(calibrationResultYEdit_[i]);
    calibrationResultLayout->addLayout(calibrationResultLineLayout);
  }
  calibrationResultGroup->setLayout(calibrationResultLayout);

  calibrationLayout->addWidget(calibrationResultGroup);
  calibrationGroup->setLayout(calibrationLayout);

  //Robot Position Group
  QGroupBox* robotPositionGroup = new QGroupBox(tr("Robot Position(x, y)/m"));
  QVBoxLayout* robotPositionLayout = new QVBoxLayout;

  for(unsigned int i=0; i<MAX_CALIBRATION_POINT_COUNT; i++)
  {
    QHBoxLayout* robotPositionLineLayout = new QHBoxLayout;
    robotPositionXEdit_[i] = new QLineEdit();
    robotPositionYEdit_[i] = new QLineEdit();

    robotPositionLineLayout->addWidget(robotPositionXEdit_[i]);
    robotPositionLineLayout->addWidget(robotPositionYEdit_[i]);
    robotPositionLayout->addLayout(robotPositionLineLayout);
  }
  robotPositionGroup->setLayout(robotPositionLayout);

  //Pixel Position Group
  QGroupBox* pixelPositionGroup = new QGroupBox(tr("Pixel Position(x, y)"));
  QVBoxLayout* pixelPositionLayout = new QVBoxLayout;

  for(unsigned int i=0; i<MAX_CALIBRATION_POINT_COUNT; i++)
  {
    QHBoxLayout* pixelPositionLineLayout = new QHBoxLayout;
    pixelPositionXEdit_[i] = new QLineEdit();
    pixelPositionYEdit_[i] = new QLineEdit();

    pixelPositionXEdit_[i]->setEnabled(false);
    pixelPositionYEdit_[i]->setEnabled(false);

    pixelPositionLineLayout->addWidget(pixelPositionXEdit_[i]);
    pixelPositionLineLayout->addWidget(pixelPositionYEdit_[i]);
    pixelPositionLayout->addLayout(pixelPositionLineLayout);
  }
  pixelPositionGroup->setLayout(pixelPositionLayout);

  // panel
  QHBoxLayout* layout = new QHBoxLayout;
  layout->addWidget(calibrationGroup);
  layout->addWidget(robotPositionGroup);
  layout->addWidget(pixelPositionGroup);

  setLayout(layout);

  connect(patternRecognitionBtn_, SIGNAL(clicked(bool)), this, SLOT(onPatternRecognitionBtnClicked(bool)));
  connect(imageCalibrationBtn_, SIGNAL(clicked(bool)), this, SLOT(onImageCalibrationUpdateBtnClicked(bool)));

  robotVisionInterface_ = RobotVisionInterfacePtr(new RobotVisionInterface(nh_));
}

void HandEyeCalibrationWidget::onPatternRecognitionBtnClicked(bool checked)
{
  std::string imageTopic = imageTopicEdit_->text().toStdString();

  ROS_INFO("开始识别标定点 %s", imageTopic.c_str());

  std::vector<float> pixelPositionX(MAX_CALIBRATION_POINT_COUNT, 0);
  std::vector<float> pixelPositionY(MAX_CALIBRATION_POINT_COUNT, 0);
  
  if(!robotVisionInterface_->patternRecognitionProcess(imageTopic, pixelPositionX, pixelPositionY))
  {
    ROS_ERROR("标定点识别失败");
    return;
  }

  for(unsigned int i=0; i<MAX_CALIBRATION_POINT_COUNT; i++)
  {
    pixelPositionXEdit_[i]->setText(QString("%1").arg(pixelPositionX[i]));
    pixelPositionYEdit_[i]->setText(QString("%1").arg(pixelPositionY[i]));
  }

  ROS_INFO("标定点识别完成");
}

void HandEyeCalibrationWidget::onImageCalibrationUpdateBtnClicked(bool checked)
{
  std::vector<float> robotPositionX(MAX_CALIBRATION_POINT_COUNT, 0);
  std::vector<float> robotPositionY(MAX_CALIBRATION_POINT_COUNT, 0);
  std::vector<float> pixelPositionX(MAX_CALIBRATION_POINT_COUNT, 0);
  std::vector<float> pixelPositionY(MAX_CALIBRATION_POINT_COUNT, 0);

  std::string calibrationyaml = calibrationDirectoryEdit_->text().toStdString();

  for(unsigned int i=0; i<MAX_CALIBRATION_POINT_COUNT; i++)
  {
    if(robotPositionXEdit_[i]->text().isEmpty() || robotPositionYEdit_[i]->text().isEmpty()
    || pixelPositionXEdit_[i]->text().isEmpty() || pixelPositionYEdit_[i]->text().isEmpty())
    {
      ROS_ERROR("标定数据不够： [%d]", i);
      return;
    }
    else
    {
      robotPositionX[i] = robotPositionXEdit_[i]->text().toFloat();
      robotPositionY[i] = robotPositionYEdit_[i]->text().toFloat();
      pixelPositionX[i] = pixelPositionXEdit_[i]->text().toFloat();
      pixelPositionY[i] = pixelPositionYEdit_[i]->text().toFloat();
    }
  }

  ROS_INFO("开始图像标定处理");

  std::vector<float> calibrationResultX(3, 0);
  std::vector<float> calibrationResultY(3, 0);

  if(!robotVisionInterface_->imageCalibrationProcess(robotPositionX, robotPositionY, pixelPositionX, pixelPositionY, calibrationResultX, calibrationResultY, calibrationyaml))
  {
    ROS_ERROR("标定失败");
    return;
  }

  for(unsigned int i=0; i<3; i++)
  {
    calibrationResultXEdit_[i]->setText(QString("%1").arg(calibrationResultX[i]));
    calibrationResultYEdit_[i]->setText(QString("%1").arg(calibrationResultY[i]));
  }

  ROS_INFO("图像标定完成");
}


} // end namespace probot_rviz_plugin
