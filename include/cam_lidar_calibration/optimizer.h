#include <string>
#include <iostream>
#include <fstream>
#include "cam_lidar_calibration/calibration_data.h"
#include "point_xyzir.h"
#include "openga.h"

using std::string;
using std::cout;
using std::endl;

class optimizer_class
{
public:
  optimizer_class();

  std::vector<double> rotm2eul(cv::Mat mat);
  void get_samples(const cam_lidar_calibration::calibration_data::ConstPtr &data);
  void my_flag(const std_msgs::Int8::ConstPtr& msg);
  void optimize();

private:
  ros::Subscriber calibdata_sub;
  int sample = 0;
  cv::Mat distcoeff = (cv::Mat_<double>(1,4) << -0.05400957120448697, -0.07842753582468161, 0.09596410068935728, -0.05152529532743679);
  cv::Mat cameramat = (cv::Mat_<double>(3,3) << 1176.931662006163, 0.0, 962.2754188883206, 0.0, 1177.715660133758, 612.7245350750861, 0.0, 0.0, 1.0);

  struct CameraVelodyneCalibrationData
  {
    std::vector<std::vector< double >> velodynenormals;
    std::vector<std::vector< double >> velodynepoints;
    std::vector<std::vector< double >> cameranormals;
    std::vector<std::vector< double >> camerapoints;
    std::vector<std::vector< double >> velodynecorners;
    std::vector<double> pixeldata;

    cv::Mat cameranormals_mat; //n*3
    cv::Mat camerapoints_mat; //n*3
    cv::Mat velodynepoints_mat; //n*3
    cv::Mat velodynenormals_mat; //n*3
    cv::Mat velodynecorners_mat; //n*3
    cv::Mat pixeldata_mat;
  }calibrationdata;
};
