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

#include <actionlib/server/simple_action_server.h>
#include <cam_lidar_calibration/Optimise.h>
#include <cam_lidar_calibration/RunOptimiseAction.h>
#include <cam_lidar_calibration/boundsConfig.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

#include "cam_lidar_calibration/load_params.h"
#include "cam_lidar_calibration/optimiser.h"
#include "cam_lidar_calibration/point_xyzir.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZIR>> pc_sub_type;

namespace cam_lidar_calibration
{
geometry_msgs::Quaternion normalToQuaternion(const cv::Point3d& normal);

class FeatureExtractor
{
public:
  FeatureExtractor();
  ~FeatureExtractor() = default;

  void extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& img,
                               const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& pc);
  bool serviceCB(Optimise::Request& req, Optimise::Response& res);

  void optimise(const RunOptimiseGoalConstPtr& goal,
                actionlib::SimpleActionServer<cam_lidar_calibration::RunOptimiseAction>* as);

  void visualiseSamples();

  void boundsCB(cam_lidar_calibration::boundsConfig& config, uint32_t level);

  bool import_samples;

private:
  void passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pc,
                   pcl::PointCloud<pcl::PointXYZIR>::Ptr& output_pc);

  std::tuple<std::vector<cv::Point3d>, cv::Mat> locateChessboard(const sensor_msgs::Image::ConstPtr& image);

  auto chessboardProjection(const std::vector<cv::Point2d>& corners, const cv_bridge::CvImagePtr& cv_ptr);

  void publishBoardPointCloud();

  std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>

  extractBoard(const pcl::PointCloud<pcl::PointXYZIR>::Ptr& cloud, OptimisationSample& sample);

  std::pair<pcl::ModelCoefficients, pcl::ModelCoefficients>
  findEdges(const pcl::PointCloud<pcl::PointXYZIR>::Ptr& edge_pair_cloud);

  void callback_camerainfo(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void distoffset_passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pc,
                              pcl::PointCloud<pcl::PointXYZIR>::Ptr& output_pc);

  std::string getDateTime();

  int find_octant(float x, float y, float z);

  std::shared_ptr<Optimiser> optimiser_;
  initial_parameters_t i_params_;
  double metreperpixel_cbdiag_;
  std::string lidar_frame_;
  std::string import_path_;
  int num_lowestvoq_;
  double distance_offset_;

  int flag = 0;
  cam_lidar_calibration::boundsConfig bounds_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZIR>>
      ImageLidarSyncPolicy;

  std::shared_ptr<image_sub_type> image_sub_;
  std::shared_ptr<pc_sub_type> pc_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
  int queue_rate_ = 10;  // This was 5 before but I changed to 10 cause
                         // Robosense to camera A0 has big timestamp misalign
  int num_samples_ = 0;

  std::vector<pcl::PointCloud<pcl::PointXYZIR>::Ptr> pc_samples_;
  ros::Publisher board_cloud_pub_, subtracted_cloud_pub_, experimental_region_pub_;
  ros::Publisher samples_pub_;
  image_transport::Publisher image_publisher_;
  ros::ServiceServer optimise_service_;
  ros::Subscriber camera_info_sub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::ImageTransport> it_p_;
  boost::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server_;

  std::string curdatetime_;
  std::string newdata_folder_;
  bool valid_camera_info_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle public_nh_;

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
