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

#include <cam_lidar_calibration/boundsConfig.h>
#include "cam_lidar_calibration/calibration_data.h"

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

namespace extrinsic_calibration
{
class FeatureExtractor : public nodelet::Nodelet
{
public:
  FeatureExtractor() = default;
  ~FeatureExtractor() = default;

  void extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::PointCloud2::ConstPtr& pc);
  void flagCB(const std_msgs::Int8::ConstPtr& msg);
  void boundsCB(cam_lidar_calibration::boundsConfig& config, uint32_t level);
  double* convertToImagePoints(double x, double y, double z);

  void bypassInit()
  {
    this->onInit();
  }

private:
  virtual void onInit();

  struct initial_parameters
  {
    std::string camera_topic;
    std::string lidar_topic;
    bool fisheye_model;
    int lidar_ring_count;
    std::pair<int, int> grid_size;
    int square_length;                         // in millimetres
    std::pair<int, int> board_dimension;       // in millimetres
    std::pair<int, int> cb_translation_error;  // in millimetres
    cv::Mat cameramat;
    int distcoeff_num;
    cv::Mat distcoeff;
  } i_params;

  std::string pkg_loc;
  int cb_l, cb_b, l, b, e_l, e_b;
  double diagonal;

  ros::Publisher pub;
  cv::FileStorage fs;
  int flag = 0;
  cam_lidar_calibration::boundsConfig bound;
  cam_lidar_calibration::calibration_data sample_data;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy>* sync;
  int queue_rate = 5;

  ros::Publisher roi_publisher;
  ros::Publisher pub_cloud, expt_region;
  ros::Publisher vis_pub, visPub;
  image_transport::Publisher image_publisher;
  ros::Subscriber flag_subscriber;
  message_filters::Subscriber<sensor_msgs::Image>* image_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pcl_sub;
  visualization_msgs::Marker marker;
  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::ImageTransport> it_p_;
  boost::shared_ptr<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>> server;
};

// PLUGINLIB_EXPORT_CLASS(extrinsic_calibration::FeatureExtractor, nodelet::Nodelet);
}  // namespace extrinsic_calibration
