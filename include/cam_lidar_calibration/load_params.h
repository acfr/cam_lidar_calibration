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

#ifndef load_params_h_
#define load_params_h_

#include <ros/ros.h>

#include <opencv2/core/mat.hpp>

namespace cam_lidar_calibration
{
    struct initial_parameters_t
    {
        bool fisheye_model;
        int lidar_ring_count = 0;
        cv::Size chessboard_pattern_size;
        int square_length;                 // in millimetres
        cv::Size board_dimensions;         // in millimetres
        cv::Point3d cb_translation_error;  // in millimetres
        cv::Mat cameramat, distcoeff;
        std::pair<int, int> image_size;  // in pixels
        std::string camera_topic, camera_info, lidar_topic;
    };

    void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params);

}  // namespace cam_lidar_calibration

#endif
