#include <QVBoxLayout>

#include "cam_lidar_panel.h"
#include <cam_lidar_calibration/Sample.h>

namespace cam_lidar_calibration
{
CamLidarPanel::CamLidarPanel(QWidget* parent) : rviz::Panel(parent)
{
  sample_client_ = nh_.serviceClient<cam_lidar_calibration::Sample>("sample");
  optimise_client_ = nh_.serviceClient<cam_lidar_calibration::Sample>("optimise");

  QVBoxLayout* button_layout = new QVBoxLayout;
  capture_button_ = new QPushButton("Capture sample");
  connect(capture_button_, SIGNAL(clicked()), this, SLOT(captureSample()));
  discard_button_ = new QPushButton("Discard last sample");
  connect(discard_button_, SIGNAL(clicked()), this, SLOT(discardSample()));
  optimise_button_ = new QPushButton("Optimise");
  connect(optimise_button_, SIGNAL(clicked()), this, SLOT(optimise()));

  button_layout->addWidget(capture_button_);
  button_layout->addWidget(discard_button_);
  button_layout->addWidget(optimise_button_);

  setLayout(button_layout);
}

void CamLidarPanel::captureSample()
{
  Sample srv;
  srv.request.operation = Sample::Request::CAPTURE;
  sample_client_.call(srv);
}

void CamLidarPanel::discardSample()
{
  Sample srv;
  srv.request.operation = Sample::Request::DISCARD;
  sample_client_.call(srv);
}

void CamLidarPanel::optimise()
{
  Sample srv;
  srv.request.operation = Sample::Request::OPTIMISE;
  optimise_client_.call(srv);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CamLidarPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void CamLidarPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

}  // end namespace cam_lidar_calibration

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cam_lidar_calibration::CamLidarPanel, rviz::Panel)
