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
 */

#ifndef cam_lidar_panel_h_
#define cam_lidar_panel_h_

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>
#endif

#include <cam_lidar_calibration/action/run_optimise.hpp>
#include <cam_lidar_calibration/srv/optimise.hpp>

#include <QLabel>
#include <QPushButton>

namespace cam_lidar_calibration
{
class CamLidarPanel : public rviz_common::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  CamLidarPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz_common::Panel functions for saving and
  // loading data from the config file.
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

public Q_SLOTS:

protected Q_SLOTS:
  void captureBackgroundPc();
  void captureSample();
  void discardSample();
  void optimise();
  void updateResult();

protected:
  // The ROS2 node
  rclcpp::Node::SharedPtr node_;
  bool import_samples_;
  rclcpp::Client<srv::Optimise>::SharedPtr optimise_client_;
  rclcpp_action::Client<action::RunOptimise>::SharedPtr action_client_;

  QLabel* output_label_;
  QPushButton* capture_background_button_;
  QPushButton* capture_button_;
  QPushButton* discard_button_;
  QPushButton* optimise_button_;
};

}  // end namespace cam_lidar_calibration

#endif  // cam_lidar_panel_h_
