#include "point_xyzir.h"
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/intersections.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/calib3d.hpp>
#include <boost/bind.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include "cam_lidar_calibration/calibration_data.h"
#include <opencv2/highgui/highgui.hpp>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "pcl_ros/transforms.h"
#include <string>
#include "opencv2/aruco.hpp"
#include <fstream>
#include <dynamic_reconfigure/server.h>
#include <cam_lidar_calibration/boundsConfig.h>
#include <ros/package.h>
#include <iostream>
#include <utility>
#include <boost/thread.hpp>

using namespace cv;
typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;

namespace extrinsic_calibration
{
class feature_extraction : public nodelet::Nodelet
{
public:
  feature_extraction();
  void extractROI(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::PointCloud2::ConstPtr& pc);
  void flag_cb(const std_msgs::Int8::ConstPtr& msg);
  void bounds_callback(cam_lidar_calibration::boundsConfig& config, uint32_t level);
  double* converto_imgpts(double x, double y, double z);

  void bypass_init()
  {
    this->onInit();
  }

private:
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

  virtual void onInit();

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

// PLUGINLIB_EXPORT_CLASS(extrinsic_calibration::feature_extraction, nodelet::Nodelet);
}  // namespace extrinsic_calibration
