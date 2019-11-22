#include "cam_lidar_calibration/feature_extractor.h"

int main(int argc, char** argv)
{
  // Initialize Node and handles
  ros::init(argc, argv, "FeatureExtractor");
  ros::NodeHandle n;

  cam_lidar_calibration::FeatureExtractor feature_extractor;
  ros::Rate loop_rate(10);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok())
  {
    feature_extractor.visualiseSamples();
    loop_rate.sleep();
  }
  return 0;
}

