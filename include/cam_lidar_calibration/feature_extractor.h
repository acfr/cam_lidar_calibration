#include <nodelet/nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>

#include <cam_lidar_calibration/Sample.h>

#include <cam_lidar_calibration/boundsConfig.h>
#include "cam_lidar_calibration/calibration_data.h"
#include "cam_lidar_calibration/load_params.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZIR>> pc_sub_type;

namespace cam_lidar_calibration
{
class FeatureExtractor : public nodelet::Nodelet
{
public:
  FeatureExtractor() = default;
  ~FeatureExtractor() = default;

  void extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& img,
                               const pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& pc);
  bool sampleCB(Sample::Request& req, Sample::Response& res);
  void boundsCB(boundsConfig& config, uint32_t level);
  double* convertToImagePoints(double x, double y, double z);

  void bypassInit()
  {
    this->onInit();
  }

private:
  virtual void onInit();

  initial_parameters_t i_params;
  std::string pkg_loc;
  int cb_l, cb_b, l, b, e_l, e_b;
  double diagonal;

  ros::Publisher pub;
  cv::FileStorage fs;
  int flag = 0;
  cam_lidar_calibration::boundsConfig bound;
  cam_lidar_calibration::calibration_data sample_data;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pcl::PointCloud<pcl::PointXYZIR>>
      ImageLidarSyncPolicy;
  std::shared_ptr<image_sub_type> image_sub_;
  std::shared_ptr<pc_sub_type> pc_sub_;
  std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> image_pc_sync_;
  int queue_rate_ = 5;

  ros::Publisher roi_publisher;
  ros::Publisher pub_cloud, expt_region;
  ros::Publisher vis_pub, visPub;
  image_transport::Publisher image_publisher;
  ros::ServiceServer sample_service_;

  visualization_msgs::Marker marker;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::ImageTransport> it_p_;
  boost::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server;
};

// PLUGINLIB_EXPORT_CLASS(extrinsic_calibration::FeatureExtractor, nodelet::Nodelet);
}  // namespace cam_lidar_calibration
