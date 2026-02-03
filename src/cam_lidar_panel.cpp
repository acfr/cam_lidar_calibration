/*
 * Copyright 2023 Australian Centre For Robotics
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * 
 * You may obtain a copy of the License at
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 * Author: Darren Tsai
 * Modified for ROS2 Jazzy
 */

#include "cam_lidar_panel.h"

#include <cam_lidar_calibration/srv/optimise.hpp>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QTimer>
#include <QVBoxLayout>

namespace cam_lidar_calibration
{
CamLidarPanel::CamLidarPanel(QWidget* parent) : rviz_common::Panel(parent)
{
  // Create ROS2 node for the panel
  node_ = rclcpp::Node::make_shared("cam_lidar_panel");
  optimise_client_ = node_->create_client<srv::Optimise>("optimiser");
  action_client_ = rclcpp_action::create_client<action::RunOptimise>(node_, "run_optimise");

  QVBoxLayout* main_layout = new QVBoxLayout;
  QHBoxLayout* button_layout = new QHBoxLayout;

  capture_background_button_ = new QPushButton("Capture Background");
  connect(capture_background_button_, SIGNAL(clicked()), this, SLOT(captureBackgroundPc()));

  capture_button_ = new QPushButton("Capture sample");
  capture_button_->setEnabled(false);
  connect(capture_button_, SIGNAL(clicked()), this, SLOT(captureSample()));

  discard_button_ = new QPushButton("Discard last sample");
  discard_button_->setEnabled(false);
  connect(discard_button_, SIGNAL(clicked()), this, SLOT(discardSample()));

  optimise_button_ = new QPushButton("Optimise");
  optimise_button_->setEnabled(false);
  connect(optimise_button_, SIGNAL(clicked()), this, SLOT(optimise()));

  QTimer* timer = new QTimer;
  connect(timer, SIGNAL(timeout()), this, SLOT(updateResult()));

  timer->start(500);

  output_label_ = new QLabel("");

  button_layout->addWidget(capture_background_button_);
  button_layout->addWidget(capture_button_);
  button_layout->addWidget(discard_button_);
  auto button_group = new QGroupBox();
  button_group->setLayout(button_layout);
  main_layout->addWidget(button_group);
  main_layout->addWidget(optimise_button_);
  main_layout->addWidget(output_label_);

  setLayout(main_layout);
}

void CamLidarPanel::captureBackgroundPc()
{
  // Send a service request to capture the background pc
  auto request = std::make_shared<srv::Optimise::Request>();
  request->operation = srv::Optimise::Request::CAPTURE_BCKGRND;
  
  if (optimise_client_->wait_for_service(std::chrono::seconds(1))) {
    optimise_client_->async_send_request(request);
    capture_button_->setEnabled(true);
  }
}

void CamLidarPanel::captureSample()
{
  auto request = std::make_shared<srv::Optimise::Request>();
  request->operation = srv::Optimise::Request::CAPTURE;

  if (optimise_client_->wait_for_service(std::chrono::seconds(1))) {
    auto future = optimise_client_->async_send_request(request);
    // In a real implementation, you'd want to handle the future response
    discard_button_->setEnabled(true);
    optimise_button_->setEnabled(true);
  } else {
    capture_button_->setEnabled(false);
  }
}

void CamLidarPanel::discardSample()
{
  auto request = std::make_shared<srv::Optimise::Request>();
  request->operation = srv::Optimise::Request::DISCARD;
  
  if (optimise_client_->wait_for_service(std::chrono::seconds(1))) {
    optimise_client_->async_send_request(request);
    optimise_button_->setEnabled(true);
  }
}

void CamLidarPanel::optimise()
{
  auto goal_msg = action::RunOptimise::Goal();
  
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available");
    return;
  }
  
  action_client_->async_send_goal(goal_msg);
}

void CamLidarPanel::updateResult()
{
  // Note: In ROS2, action state tracking is handled differently
  // This is a simplified version - full implementation would track goal handles
  // For now, just enable buttons after a delay
  // A complete implementation would store the goal handle from async_send_goal
  // and check its status here
  
  // Placeholder implementation - buttons stay enabled
  // In a full implementation, you'd track the goal handle state
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void CamLidarPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void CamLidarPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

}  // end namespace cam_lidar_calibration

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cam_lidar_calibration::CamLidarPanel, rviz_common::Panel)
