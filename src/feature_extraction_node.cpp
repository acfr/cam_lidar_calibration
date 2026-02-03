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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cam_lidar_calibration/action/run_optimise.hpp"
#include "cam_lidar_calibration/feature_extractor.h"

using cam_lidar_calibration::FeatureExtractor;

int main(int argc, char** argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create node
  auto node = std::make_shared<rclcpp::Node>("feature_extractor");
  
  // Create feature extractor
  auto feature_extractor = std::make_shared<FeatureExtractor>(node);
  
  // Create action server
  using RunOptimise = cam_lidar_calibration::action::RunOptimise;
  auto action_server = rclcpp_action::create_server<RunOptimise>(
    node,
    "run_optimise",
    [](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const RunOptimise::Goal> goal) {
      (void)uuid;
      (void)goal;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunOptimise>> goal_handle) {
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [feature_extractor](const std::shared_ptr<rclcpp_action::ServerGoalHandle<RunOptimise>> goal_handle) {
      feature_extractor->optimise(goal_handle);
    }
  );
  
  // Create executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  
  // Run in separate thread to allow for visualization loop
  std::thread executor_thread([&executor]() { executor.spin(); });
  
  // Main loop for visualization
  rclcpp::Rate loop_rate(10);
  while (rclcpp::ok())
  {
    if (feature_extractor->import_samples)
    {
      using RunOptimiseAction = cam_lidar_calibration::action::RunOptimise;
      auto action_client = rclcpp_action::create_client<RunOptimiseAction>(node, "run_optimise");
      
      if (action_client->wait_for_action_server(std::chrono::seconds(10)))
      {
        auto goal_msg = RunOptimiseAction::Goal();
        action_client->async_send_goal(goal_msg);
      }
      break;
    }
    
    feature_extractor->visualiseSamples();
    loop_rate.sleep();
  }
  
  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 0;
}
