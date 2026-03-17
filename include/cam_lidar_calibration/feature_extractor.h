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
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

#include "cam_lidar_calibration/action/run_optimise.hpp"
#include "cam_lidar_calibration/srv/optimise.hpp"
#include "cam_lidar_calibration/load_params.h"
#include "cam_lidar_calibration/optimiser.h"
#include "cam_lidar_calibration/point_xyzir.h"

typedef message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_type;

namespace cam_lidar_calibration
{
geometry_msgs::msg::Quaternion normalToQuaternion(const cv::Point3d& normal);

class FeatureExtractor
{
public:
  explicit FeatureExtractor(rclcpp::Node::SharedPtr node);
  ~FeatureExtractor() = default;

  void extractRegionOfInterest(const sensor_msgs::msg::Image::ConstSharedPtr& img,
                               const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pc);
  bool serviceCB(const std::shared_ptr<srv::Optimise::Request> req, 
                 std::shared_ptr<srv::Optimise::Response> res);

  void optimise(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action::RunOptimise>> goal_handle);

  void visualiseSamples();

  void boundsCB(const rclcpp::Parameter& param);

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter>& parameters);

  bool import_samples;

private:
  void passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pc,
                   pcl::PointCloud<pcl::PointXYZIR>::Ptr& output_pc);

  std::tuple<std::vector<cv::Point3d>, cv::Mat> locateChessboard(const sensor_msgs::msg::Image::ConstSharedPtr& image);

  auto chessboardProjection(const std::vector<cv::Point2d>& corners, const cv_bridge::CvImagePtr& cv_ptr);

  void publishBoardPointCloud();

  std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>

  extractBoard(const pcl::PointCloud<pcl::PointXYZIR>::Ptr& cloud, OptimisationSample& sample);

  std::pair<pcl::ModelCoefficients, pcl::ModelCoefficients>
  findEdges(const pcl::PointCloud<pcl::PointXYZIR>::Ptr& edge_pair_cloud);

  void callback_camerainfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);

  void distoffset_passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pc,
                              pcl::PointCloud<pcl::PointXYZIR>::Ptr& output_pc);

  std::string getDateTime();

  int find_octant(float x, float y, float z);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Optimiser> optimiser_;
  initial_parameters_t i_params_;
  double metreperpixel_cbdiag_;
  std::string lidar_frame_;
  std::string import_path_;
  int num_lowestvoq_;
  double distance_offset_;

  int flag = 0;
  // ROS2 Note: bounds configuration will use parameters instead of dynamic_reconfigure
  struct BoundsConfig {
    double x_min, x_max, y_min, y_max, z_min, z_max;
    int k;  // for statistical outlier removal mean K
    double z;  // for statistical outlier removal stddev threshold
    double voxel_res;  // voxel resolution for octree
  } bounds_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2>
      ImageLidarSyncPolicy;

  std::shared_ptr<image_sub_type> image_sub_;
  std::shared_ptr<pc_sub_type> pc_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
  int queue_rate_ = 10;  // This was 5 before but I changed to 10 cause
                         // Robosense to camera A0 has big timestamp misalign
  int num_samples_ = 0;

  std::vector<pcl::PointCloud<pcl::PointXYZIR>::Ptr> pc_samples_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr board_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr subtracted_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr experimental_region_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr samples_pub_;
  image_transport::Publisher image_publisher_;
  rclcpp::Service<srv::Optimise>::SharedPtr optimise_service_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  std::shared_ptr<image_transport::ImageTransport> it_p_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  std::string curdatetime_;
  std::string newdata_folder_;
  bool valid_camera_info_;

  std::vector<pcl::PointCloud<pcl::PointXYZIR>::Ptr> background_pc_samples_;
  double board_width_ = 0.0f;
  double board_height_ = 0.0f;

  int num_of_pc_frames_;
  int frames_to_capture_ = 5;
  int num_invalid_samples_ = 0;

  // five consecutive frames are captured per caputre button press
  std::vector<cam_lidar_calibration::OptimisationSample> samples_;
};

}  // namespace cam_lidar_calibration
