#include "cam_lidar_calibration/feature_extractor.h"

#include <actionlib/server/simple_action_server.h>
#include <cam_lidar_calibration/RunOptimiseAction.h>

using actionlib::SimpleActionServer;
using cam_lidar_calibration::FeatureExtractor;

int main(int argc, char** argv)
{
  // Initialize Node and handles
  ros::init(argc, argv, "FeatureExtractor");
  ros::NodeHandle n;

  FeatureExtractor feature_extractor;
  SimpleActionServer<cam_lidar_calibration::RunOptimiseAction> optimise_action(
      n, "run_optimise", boost::bind(&FeatureExtractor::optimise, feature_extractor, _1, &optimise_action), false);
  optimise_action.start();

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

