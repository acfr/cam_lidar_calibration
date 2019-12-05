#include "cam_lidar_calibration/load_params.h"

namespace cam_lidar_calibration
{
void loadParams(const ros::NodeHandle& n, initial_parameters_t& i_params)
{
  int cb_w, cb_h, w, h, e_x, e_y, i_l, i_b;
  n.getParam("fisheye_model", i_params.fisheye_model);
  n.getParam("chessboard/pattern_size/width", cb_w);
  n.getParam("chessboard/pattern_size/height", cb_h);
  i_params.chessboard_pattern_size = cv::Size(cb_w, cb_h);
  n.getParam("chessboard/square_length", i_params.square_length);
  n.getParam("chessboard/board_dimension/width", w);
  n.getParam("chessboard/board_dimension/height", h);
  i_params.board_dimensions = cv::Size(w, h);
  n.getParam("chessboard/translation_error/x", e_x);
  n.getParam("chessboard/translation_error/y", e_y);
  i_params.cb_translation_error = cv::Point3d(e_x, e_y, 0);
  std::vector<double> camera_mat;
  n.getParam("cameramat", camera_mat);
  cv::Mat(3, 3, CV_64F, camera_mat.data()).copyTo(i_params.cameramat);
  std::vector<double> dist_coeff;
  n.getParam("distcoeff", dist_coeff);
  cv::Mat(1, dist_coeff.size(), CV_64F, dist_coeff.data()).copyTo(i_params.distcoeff);
  n.getParam("image_size/l", i_l);
  n.getParam("image_size/b", i_b);
  i_params.image_size = std::make_pair(i_l, i_b);
}
}  // namespace cam_lidar_calibration

