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

#include "cam_lidar_calibration/load_params.h"

namespace cam_lidar_calibration
{
void loadParams(rclcpp::Node::SharedPtr node, initial_parameters_t& i_params_)
{
  // Declare and get parameters
  i_params_.camera_topic = node->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
  i_params_.camera_info = node->declare_parameter<std::string>("camera_info", "/camera/camera_info");
  i_params_.lidar_topic = node->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
  
  int cb_w = node->declare_parameter<int>("chessboard.pattern_size.width", 8);
  int cb_h = node->declare_parameter<int>("chessboard.pattern_size.height", 6);
  i_params_.chessboard_pattern_size = cv::Size(cb_w, cb_h);
  
  i_params_.square_length = node->declare_parameter<int>("chessboard.square_length", 100);
  
  int w = node->declare_parameter<int>("chessboard.board_dimension.width", 800);
  int h = node->declare_parameter<int>("chessboard.board_dimension.height", 600);
  i_params_.board_dimensions = cv::Size(w, h);
  
  int e_x = node->declare_parameter<int>("chessboard.translation_error.x", 0);
  int e_y = node->declare_parameter<int>("chessboard.translation_error.y", 0);
  i_params_.cb_translation_error = cv::Point3d(e_x, e_y, 0);
}
}  // namespace cam_lidar_calibration
