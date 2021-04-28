#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>

#include "probot_rviz_plugin/widget/pick_and_place_widget.h"

namespace probot_rviz_plugin
{

PickAndPlaceWidget::PickAndPlaceWidget(QLineEdit  *outImageTopicEdit, QWidget* parent)
  : outImageTopicEdit_(outImageTopicEdit), QWidget(parent)
{

  QHBoxLayout* pixelPositionLayout = new QHBoxLayout;
  QHBoxLayout* objectPositionLayout = new QHBoxLayout;
  QHBoxLayout* placePositionLayout = new QHBoxLayout;

  pixelPositionXEdit_ = new QLineEdit("");
  pixelPositionYEdit_ = new QLineEdit("");
  pixelPositionZEdit_ = new QLineEdit("--");

  pixelPositionXEdit_->setEnabled(false);
  pixelPositionYEdit_->setEnabled(false);
  pixelPositionZEdit_->setEnabled(false);

  pixelPositionLayout->addWidget(new QLabel(tr("Pixel Position : ")));
  pixelPositionLayout->addWidget(new QLabel(tr("X-")));
  pixelPositionLayout->addWidget(pixelPositionXEdit_);
  pixelPositionLayout->addWidget(new QLabel(tr("Y-")));
  pixelPositionLayout->addWidget(pixelPositionYEdit_);
  pixelPositionLayout->addWidget(new QLabel(tr("Z-")));
  pixelPositionLayout->addWidget(pixelPositionZEdit_);

  
  objectPositionXEdit_ = new QLineEdit("");
  objectPositionYEdit_ = new QLineEdit("");
  objectPositionZEdit_ = new QLineEdit("");

  objectPositionXEdit_->setEnabled(false);
  objectPositionYEdit_->setEnabled(false);

  objectPositionLayout->addWidget(new QLabel(tr("Object Position : ")));
  objectPositionLayout->addWidget(new QLabel(tr("X-")));
  objectPositionLayout->addWidget(objectPositionXEdit_);
  objectPositionLayout->addWidget(new QLabel(tr("Y-")));
  objectPositionLayout->addWidget(objectPositionYEdit_);
  objectPositionLayout->addWidget(new QLabel(tr("Z-")));
  objectPositionLayout->addWidget(objectPositionZEdit_);


  placePositionXEdit_ = new QLineEdit("");
  placePositionYEdit_ = new QLineEdit("");
  placePositionZEdit_ = new QLineEdit("");

  placePositionLayout->addWidget(new QLabel(tr("Place Position : ")));
  placePositionLayout->addWidget(new QLabel(tr("X-")));
  placePositionLayout->addWidget(placePositionXEdit_);
  placePositionLayout->addWidget(new QLabel(tr("Y-")));
  placePositionLayout->addWidget(placePositionYEdit_);
  placePositionLayout->addWidget(new QLabel(tr("Z-")));
  placePositionLayout->addWidget(placePositionZEdit_);


  QHBoxLayout* pickAndPlaceButtonLayout = new QHBoxLayout;
  objectRecognitionBtn_ = new QPushButton(tr("Object Recognition"));
  pickAndPlaceStartBtn_ = new QPushButton(tr("Pick and Place!"));

  connect(objectRecognitionBtn_, SIGNAL(clicked(bool)), this, SLOT(onObjectRecognitionBtnClicked(bool)));
  connect(pickAndPlaceStartBtn_, SIGNAL(clicked(bool)), this, SLOT(onPickAndPlaceBtnClicked(bool)));

  pickAndPlaceButtonLayout->addWidget(objectRecognitionBtn_);
  pickAndPlaceButtonLayout->addWidget(pickAndPlaceStartBtn_);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(pixelPositionLayout);
  layout->addLayout(objectPositionLayout);
  layout->addLayout(placePositionLayout);
  layout->addLayout(pickAndPlaceButtonLayout);

  setLayout(layout);
  
  robotVisionInterface_ = RobotVisionInterfacePtr(new RobotVisionInterface(nh_));
}

void PickAndPlaceWidget::onObjectRecognitionBtnClicked(bool checked)
{
  std::vector<unsigned int> pixelPosition(3, 0);
  std::vector<float> objectPosition(3, 0.0);

  if(robotVisionInterface_->objectRecognitionProcess(pixelPosition, objectPosition))
  {
    pixelPositionXEdit_->setText(QString("%1").arg(pixelPosition[0]));
    pixelPositionYEdit_->setText(QString("%1").arg(pixelPosition[1]));

    objectPositionXEdit_->setText(QString("%1").arg(objectPosition[0]));
    objectPositionYEdit_->setText(QString("%1").arg(objectPosition[1]));
  }
  else
  {
    ROS_ERROR("目标识别失败!");
  }
}

void PickAndPlaceWidget::onPickAndPlaceBtnClicked(bool checked)
{
  std::vector<float> pickPosition(3, 0.0);
  std::vector<float> placePosition(3, 0.0);

  if(objectPositionXEdit_->text().isEmpty() || 
    objectPositionYEdit_->text().isEmpty()  ||
    objectPositionZEdit_->text().isEmpty()  ||
    placePositionXEdit_->text().isEmpty()   ||
    placePositionYEdit_->text().isEmpty()   ||
    placePositionZEdit_->text().isEmpty() )
  {
    ROS_ERROR("抓取或摆放目标位置未设置!");
    return;
  }

  pickPosition[0] = objectPositionXEdit_->text().toFloat();
  pickPosition[1] = objectPositionYEdit_->text().toFloat();
  pickPosition[2] = objectPositionZEdit_->text().toFloat();

  placePosition[0] = placePositionXEdit_->text().toFloat();
  placePosition[1] = placePositionYEdit_->text().toFloat();
  placePosition[2] = placePositionZEdit_->text().toFloat();

  if(robotVisionInterface_->pickAndPlaceProcess(pickPosition, placePosition))
  {
    ROS_INFO("完成抓取!");
  }
  else
  {
    ROS_ERROR("抓取失败!");
  }
}

} // end namespace probot_rviz_plugin
