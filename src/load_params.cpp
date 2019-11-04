#include "cam_lidar_calibration/load_params.h"

namespace cam_lidar_calibration
{
void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params)
{
  int cb_l, cb_b, l, b, e_l, e_b, i_l, i_b;
  n.getParam("image_topic", i_params.camera_topic);
  n.getParam("lidar_topic", i_params.lidar_topic);
  n.getParam("fisheye_model", i_params.fisheye_model);
  n.getParam("lidar_ring_count", i_params.lidar_ring_count);
  n.getParam("grid_size/l", cb_l);
  n.getParam("grid_size/b", cb_b);
  i_params.grid_size = std::make_pair(cb_l, cb_b);
  n.getParam("square_length", i_params.square_length);
  n.getParam("board_dimension/l", l);
  n.getParam("board_dimension/b", b);
  i_params.board_dimension = std::make_pair(l, b);
  n.getParam("cb_translation_error/l", e_l);
  n.getParam("cb_translation_error/b", e_b);
  i_params.cb_translation_error = std::make_pair(e_l, e_b);
  std::vector<double> camera_mat;
  n.getParam("cameramat", camera_mat);
  cv::Mat(3, 3, CV_64F, &camera_mat).copyTo(i_params.cameramat);
  n.getParam("distcoeff_num", i_params.distcoeff_num);
  std::vector<double> dist_coeff;
  n.getParam("distcoeff", dist_coeff);
  cv::Mat(1, i_params.distcoeff_num, CV_64F, &dist_coeff).copyTo(i_params.distcoeff);
  n.getParam("image_size/l", i_l);
  n.getParam("image_size/b", i_b);
  i_params.image_size = std::make_pair(i_l, i_b);
  /*
  std::string pkg_loc = ros::package::getPath("cam_lidar_calibration");
  std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");

  infile >> i_params.camera_topic;
  infile >> i_params.lidar_topic;
  infile >> i_params.fisheye_model;
  infile >> i_params.lidar_ring_count;
  infile >> cb_l;
  infile >> cb_b;
  i_params.grid_size = std::make_pair(cb_l, cb_b);
  infile >> i_params.square_length;
  infile >> l;
  infile >> b;
  i_params.board_dimension = std::make_pair(l, b);
  infile >> e_l;
  infile >> e_b;
  i_params.cb_translation_error = std::make_pair(e_l, e_b);
  double camera_mat[9];
  for (int i = 0; i < 9; i++)
  {
    infile >> camera_mat[i];
  }
  cv::Mat(3, 3, CV_64F, &camera_mat).copyTo(i_params.cameramat);

  infile >> i_params.distcoeff_num;
  double dist_coeff[i_params.distcoeff_num];
  for (int i = 0; i < i_params.distcoeff_num; i++)
  {
    infile >> dist_coeff[i];
  }
  cv::Mat(1, i_params.distcoeff_num, CV_64F, &dist_coeff).copyTo(i_params.distcoeff);
  infile >> i_l;
  infile >> i_b;
  i_params.image_size = std::make_pair(i_l, i_b);
  */
}
}  // namespace cam_lidar_calibration

