#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>

#include "probot_rviz_plugin/widget/image_process_config_widget.h"

namespace probot_rviz_plugin
{

ImageProcessConfigWidget::ImageProcessConfigWidget(QLineEdit *rawImageTopicEdit, QLineEdit *outImageTopicEdit, QWidget* parent)
  : rawImageTopicEdit_(rawImageTopicEdit), outImageTopicEdit_(outImageTopicEdit), QWidget(parent)
{
  //ROI Parameters
  QGroupBox* roiParameterGroup = new QGroupBox(tr("ROI Parameters"));
  QVBoxLayout* roiParameterGroupLayout = new QVBoxLayout;

  QHBoxLayout* roiParameterLayout = new QHBoxLayout;

  roiXEdit_ = new QLineEdit("0");
  roiYEdit_ = new QLineEdit("0");
  roiHeightEdit_ = new QLineEdit("480");
  roiWidthEdit_  = new QLineEdit("640");
  
  roiSetBtn_ = new QPushButton(tr("Set"));
  connect(roiSetBtn_, SIGNAL(clicked(bool)), this, SLOT(onRoiSetBtnClicked(bool)));

  roiParameterLayout->addWidget(new QLabel(tr(" X:")));
  roiParameterLayout->addWidget(roiXEdit_);

  roiParameterLayout->addWidget(new QLabel(tr(" Y:")));
  roiParameterLayout->addWidget(roiYEdit_);

  roiParameterLayout->addWidget(new QLabel(tr(" Height:")));
  roiParameterLayout->addWidget(roiHeightEdit_);

  roiParameterLayout->addWidget(new QLabel(tr(" Width:")));
  roiParameterLayout->addWidget(roiWidthEdit_);

  roiParameterGroupLayout->addLayout(roiParameterLayout);
  roiParameterGroupLayout->addWidget(roiSetBtn_);

  roiParameterGroup->setLayout(roiParameterGroupLayout);


   //param save
  QGroupBox* paramGroup = new QGroupBox(tr("param save"));
  QVBoxLayout* paramGroupLayout = new QVBoxLayout;

  QHBoxLayout* parameterLayout = new QHBoxLayout;

  directoryEdit_ = new QLineEdit("/home/tianbot/xarm_ws/src/probot_rviz_plugin/config/roi_hsv_config.yaml");
  
  paramSaveBtn_ = new QPushButton(tr("Save"));
  connect(paramSaveBtn_, SIGNAL(clicked(bool)), this, SLOT(onParamSaveBtnClicked(bool)));

  parameterLayout->addWidget(new QLabel(tr("dir:")));
  parameterLayout->addWidget(directoryEdit_);

  paramGroupLayout->addLayout(parameterLayout);
  paramGroupLayout->addWidget(paramSaveBtn_);

  paramGroup->setLayout(paramGroupLayout);

  //HSV parameters
  QGroupBox* hsvParameterGroup = new QGroupBox(tr("HSV Parameters"));
  QVBoxLayout* hsvParameterLayout = new QVBoxLayout;

  //h
  hmaxValueSlider_ = new QSlider(Qt::Horizontal, this);
  hmaxValueSlider_->setMinimum(0);
  hmaxValueSlider_->setMaximum(360);
  hmaxValueSlider_->setValue(360); 
  hmaxValueLabel_ = new QLabel("(360)");
  connect(hmaxValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(hmaxValueSliderValueChanged(int)));

  QHBoxLayout* hmaxValueLayout = new QHBoxLayout;
  hmaxValueLayout->addWidget(new QLabel(tr("hmax")));
  hmaxValueLayout->addWidget(hmaxValueSlider_);
  hmaxValueLayout->addWidget(hmaxValueLabel_);

  hminValueSlider_ = new QSlider(Qt::Horizontal, this);
  hminValueSlider_->setMinimum(0);
  hminValueSlider_->setMaximum(360);
  hminValueSlider_->setValue(0); 
  hminValueLabel_ = new QLabel("(0)");
  connect(hminValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(hminValueSliderValueChanged(int)));

  QHBoxLayout* hminValueLayout = new QHBoxLayout;
  hminValueLayout->addWidget(new QLabel(tr("hmin")));
  hminValueLayout->addWidget(hminValueSlider_);
  hminValueLayout->addWidget(hminValueLabel_);

  //s
  smaxValueSlider_ = new QSlider(Qt::Horizontal, this);
  smaxValueSlider_->setMinimum(0);
  smaxValueSlider_->setMaximum(255);
  smaxValueSlider_->setValue(255); 
  smaxValueLabel_ = new QLabel("(255)");
  connect(smaxValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(smaxValueSliderValueChanged(int)));

  QHBoxLayout* smaxValueLayout = new QHBoxLayout;
  smaxValueLayout->addWidget(new QLabel(tr("smax")));
  smaxValueLayout->addWidget(smaxValueSlider_);
  smaxValueLayout->addWidget(smaxValueLabel_);

  sminValueSlider_ = new QSlider(Qt::Horizontal, this);
  sminValueSlider_->setMinimum(0);
  sminValueSlider_->setMaximum(255);
  sminValueSlider_->setValue(70); 
  sminValueLabel_ = new QLabel("(70)");
  connect(sminValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(sminValueSliderValueChanged(int)));

  QHBoxLayout* sminValueLayout = new QHBoxLayout;
  sminValueLayout->addWidget(new QLabel(tr("smin")));
  sminValueLayout->addWidget(sminValueSlider_);
  sminValueLayout->addWidget(sminValueLabel_);

  //v
  vmaxValueSlider_ = new QSlider(Qt::Horizontal, this);
  vmaxValueSlider_->setMinimum(0);
  vmaxValueSlider_->setMaximum(255);
  vmaxValueSlider_->setValue(250); 
  vmaxValueLabel_ = new QLabel("(250)");
  connect(vmaxValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(vmaxValueSliderValueChanged(int)));

  QHBoxLayout* vmaxValueLayout = new QHBoxLayout;
  vmaxValueLayout->addWidget(new QLabel(tr("vmax")));
  vmaxValueLayout->addWidget(vmaxValueSlider_);
  vmaxValueLayout->addWidget(vmaxValueLabel_);

  vminValueSlider_ = new QSlider(Qt::Horizontal, this);
  vminValueSlider_->setMinimum(0);
  vminValueSlider_->setMaximum(255);
  vminValueSlider_->setValue(60); 
  vminValueLabel_ = new QLabel("(60)");
  connect(vminValueSlider_, SIGNAL(valueChanged(int)), this, SLOT(vminValueSliderValueChanged(int)));

  QHBoxLayout* vminValueLayout = new QHBoxLayout;
  vminValueLayout->addWidget(new QLabel(tr("vmin")));
  vminValueLayout->addWidget(vminValueSlider_);
  vminValueLayout->addWidget(vminValueLabel_);

  hsvParameterLayout->addLayout(hminValueLayout);
  hsvParameterLayout->addLayout(hmaxValueLayout);
  hsvParameterLayout->addLayout(sminValueLayout);
  hsvParameterLayout->addLayout(smaxValueLayout);
  hsvParameterLayout->addLayout(vminValueLayout);
  hsvParameterLayout->addLayout(vmaxValueLayout);
  hsvParameterGroup->setLayout(hsvParameterLayout);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(roiParameterGroup);
  layout->addWidget(hsvParameterGroup);
  layout->addWidget(paramGroup );

  setLayout(layout);


  imageMinHsv_.resize(3);
  imageMaxHsv_.resize(3);

  imageMinHsv_[0] = hminValueSlider_->value();
  imageMinHsv_[1] = sminValueSlider_->value();
  imageMinHsv_[2] = vminValueSlider_->value();

  imageMaxHsv_[0] = hmaxValueSlider_->value();
  imageMaxHsv_[1] = smaxValueSlider_->value();
  imageMaxHsv_[2] = vmaxValueSlider_->value();

  robotVisionInterface_ = RobotVisionInterfacePtr(new RobotVisionInterface(nh_));
}

void ImageProcessConfigWidget::onRoiSetBtnClicked(bool checked)
{
  unsigned int roiX      = roiXEdit_->text().toUInt();
  unsigned int roiY      = roiYEdit_->text().toUInt();
  unsigned int roiHeight = roiHeightEdit_->text().toUInt();
  unsigned int roiWidth  = roiWidthEdit_->text().toUInt();

  robotVisionInterface_->setRoiParameters(roiX, roiY, roiHeight, roiWidth);
}

void ImageProcessConfigWidget::onParamSaveBtnClicked(bool checked)
{
  std::string yamlname = directoryEdit_->text().toStdString();

  robotVisionInterface_->saveParameters(yamlname);
}

void ImageProcessConfigWidget::hmaxValueSliderValueChanged(int value)
{
  hmaxValueLabel_->setText(QString("(%1)").arg(value));

  imageMaxHsv_[0] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

void ImageProcessConfigWidget::hminValueSliderValueChanged(int value)
{
  hminValueLabel_->setText(QString("(%1)").arg(value));

  imageMinHsv_[0] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

void ImageProcessConfigWidget::smaxValueSliderValueChanged(int value)
{
  smaxValueLabel_->setText(QString("(%1)").arg(value));

  imageMaxHsv_[1] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

void ImageProcessConfigWidget::sminValueSliderValueChanged(int value)
{
  sminValueLabel_->setText(QString("(%1)").arg(value));

  imageMinHsv_[1] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

void ImageProcessConfigWidget::vmaxValueSliderValueChanged(int value)
{
  vmaxValueLabel_->setText(QString("(%1)").arg(value));

  imageMaxHsv_[2] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

void ImageProcessConfigWidget::vminValueSliderValueChanged(int value)
{
  vminValueLabel_->setText(QString("(%1)").arg(value));

  imageMinHsv_[2] = value;
  robotVisionInterface_->setHsvParameters(imageMinHsv_, imageMaxHsv_);
}

} // end namespace probot_rviz_plugin
