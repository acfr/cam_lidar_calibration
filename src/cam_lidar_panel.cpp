#include <QVBoxLayout>

#include "cam_lidar_panel.h"
#include <cam_lidar_calibration/Optimise.h>

namespace cam_lidar_calibration
{
CamLidarPanel::CamLidarPanel(QWidget* parent) : rviz::Panel(parent)
{
  optimise_client_ = nh_.serviceClient<cam_lidar_calibration::Optimise>("optimiser");

  QVBoxLayout* button_layout = new QVBoxLayout;
  capture_button_ = new QPushButton("Capture sample");
  connect(capture_button_, SIGNAL(clicked()), this, SLOT(captureSample()));
  discard_button_ = new QPushButton("Discard last sample");
  connect(discard_button_, SIGNAL(clicked()), this, SLOT(discardSample()));
  optimise_button_ = new QPushButton("Optimise");
  optimise_button_->setEnabled(false);
  connect(optimise_button_, SIGNAL(clicked()), this, SLOT(optimise()));

  button_layout->addWidget(capture_button_);
  button_layout->addWidget(discard_button_);
  button_layout->addWidget(optimise_button_);

  setLayout(button_layout);
}

void CamLidarPanel::captureSample()
{
  Optimise srv;
  srv.request.operation = Optimise::Request::CAPTURE;
  optimise_client_.call(srv);
  if (srv.response.samples < 3)
  {
    optimise_button_->setEnabled(false);
  }
  else
  {
    optimise_button_->setEnabled(true);
  }
}

void CamLidarPanel::discardSample()
{
  Optimise srv;
  srv.request.operation = Optimise::Request::DISCARD;
  optimise_client_.call(srv);
  if (srv.response.samples < 3)
  {
    optimise_button_->setEnabled(false);
  }
  else
  {
    optimise_button_->setEnabled(true);
  }
}

void CamLidarPanel::optimise()
{
  Optimise srv;
  srv.request.operation = Optimise::Request::OPTIMISE;
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
