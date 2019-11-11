#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

#include <cam_lidar_calibration/Optimise.h>

#include <cam_lidar_calibration/boundsConfig.h>
#include "cam_lidar_calibration/calibration_data.h"
#include "cam_lidar_calibration/load_params.h"
#include "cam_lidar_calibration/optimiser.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZIR>> pc_sub_type;

namespace cam_lidar_calibration
{
geometry_msgs::Quaternion normalToQuaternion(const cv::Point3d& normal);

class FeatureExtractor : public nodelet::Nodelet
{
public:
  FeatureExtractor() = default;
  ~FeatureExtractor() = default;

  void extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& img,
                               const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& pc);
  bool serviceCB(Optimise::Request& req, Optimise::Response& res);
  void boundsCB(boundsConfig& config, uint32_t level);

  void visualiseSamples();

  void bypassInit()
  {
    this->onInit();
  }

private:
  virtual void onInit();

  void passthrough(const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pc,
                   pcl::PointCloud<pcl::PointXYZIR>::Ptr& output_pc);
  std::tuple<std::vector<cv::Point3d>, cv::Mat> locateChessboard(const sensor_msgs::Image::ConstPtr& image);
  auto chessboardProjection(const std::vector<cv::Point2d>& corners, const cv_bridge::CvImagePtr& cv_ptr);

  std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>
  extractBoard(const pcl::PointCloud<pcl::PointXYZIR>::Ptr& cloud);

  Optimiser optimiser_;
  initial_parameters_t i_params;
  int cb_l, cb_b, l, b, e_l, e_b;
  std::string lidar_frame_;

  int flag = 0;
  cam_lidar_calibration::boundsConfig bounds_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZIR>>
      ImageLidarSyncPolicy;
  std::shared_ptr<image_sub_type> image_sub_;
  std::shared_ptr<pc_sub_type> pc_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
  int queue_rate_ = 5;

  ros::Publisher board_cloud_pub_, bounded_cloud_pub_;
  ros::Publisher samples_pub_;
  image_transport::Publisher image_publisher;
  ros::ServiceServer optimise_service_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::ImageTransport> it_p_;
  boost::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server;
};

// PLUGINLIB_EXPORT_CLASS(extrinsic_calibration::FeatureExtractor, nodelet::Nodelet);
}  // namespace cam_lidar_calibration
