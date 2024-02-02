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

#ifndef POINTXYZIR_H
#define POINTXYZIR_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZIR
{
  PCL_ADD_POINT4D;
  float intensity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                                      intensity)(uint16_t, ring, ring))

#endif
