#include <ros/ros.h>

#include "cam_lidar_calibration/point_xyzir.h"
#include <pcl/point_cloud.h>
#include <pcl/common/intersections.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/impl/project_inliers.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

#include <opencv2/calib3d.hpp>

#include <Eigen/Geometry>

#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>

#include "cam_lidar_calibration/feature_extractor.h"

using cv::findChessboardCorners;
using cv::Mat_;
using cv::Size;
using cv::TermCriteria;

using PointCloud = pcl::PointCloud<pcl::PointXYZIR>;

int main(int argc, char** argv)
{
  // Initialize Node and handles
  ros::init(argc, argv, "FeatureExtractor");
  ros::NodeHandle n;

  cam_lidar_calibration::FeatureExtractor feature_extractor;
  feature_extractor.bypassInit();
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    feature_extractor.visualiseSamples();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

namespace cam_lidar_calibration
{
void FeatureExtractor::onInit()
{
  // Creating ROS nodehandle
  ros::NodeHandle private_nh = ros::NodeHandle("~");
  ros::NodeHandle public_nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");  // getMTPrivateNodeHandle();
  loadParams(public_nh, i_params);
  ROS_INFO("Input parameters loaded");

  it_.reset(new image_transport::ImageTransport(public_nh));
  it_p_.reset(new image_transport::ImageTransport(private_nh));

  // Dynamic reconfigure gui to set the experimental region bounds
  server = boost::make_shared<dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>>(pnh);
  dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>::CallbackType f;
  f = boost::bind(&FeatureExtractor::boundsCB, this, _1, _2);
  server->setCallback(f);

  // Synchronizer to get synchronized camera-lidar scan pairs
  image_sub_ = std::make_shared<image_sub_type>(private_nh, "image", queue_rate_);
  pc_sub_ = std::make_shared<pc_sub_type>(private_nh, "pointcloud", queue_rate_);
  image_pc_sync_ = std::make_shared<message_filters::Synchronizer<ImageLidarSyncPolicy>>(
      ImageLidarSyncPolicy(queue_rate_), *image_sub_, *pc_sub_);
  image_pc_sync_->registerCallback(boost::bind(&FeatureExtractor::extractRegionOfInterest, this, _1, _2));

  board_cloud_pub_ = private_nh.advertise<PointCloud>("chessboard", 1);
  bounded_cloud_pub_ = private_nh.advertise<PointCloud>("experimental_region", 10);
  optimise_service_ = public_nh.advertiseService("optimiser", &FeatureExtractor::serviceCB, this);
  samples_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("collected_samples", 0);
  image_publisher = it_->advertise("camera_features", 1);
  NODELET_INFO_STREAM("Camera Lidar Calibration");
}

bool FeatureExtractor::serviceCB(Optimise::Request& req, Optimise::Response& res)
{
  switch (req.operation)
  {
    case Optimise::Request::CAPTURE:
      ROS_INFO("Capturing sample");
      break;
    case Optimise::Request::DISCARD:
      ROS_INFO("Discarding last sample");
      if (!optimiser_.samples_.empty())
      {
        optimiser_.samples_.pop_back();
      }
      break;
    case Optimise::Request::OPTIMISE:
      ROS_INFO("Calling optimiser");
      optimiser_.optimise();
      break;
  }

  flag = req.operation;  // read flag published by rviz calibration panel
  return true;
}

void FeatureExtractor::boundsCB(cam_lidar_calibration::boundsConfig& config, uint32_t level)
{
  // Read the values corresponding to the motion of slider bars in reconfigure gui
  bounds_ = config;
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf", config.x_min, config.x_max, config.y_min, config.y_max,
           config.z_min, config.z_max);
}

geometry_msgs::Quaternion normalToQuaternion(const cv::Point3d& normal)
{
  // Convert to Eigen vector
  Eigen::Vector3d eigen_normal(normal.x, normal.y, normal.z);
  Eigen::Vector3d axis(1, 0, 0);
  auto eigen_quat = Eigen::Quaterniond::FromTwoVectors(axis, eigen_normal);
  geometry_msgs::Quaternion quat;
  quat.w = eigen_quat.w();
  quat.x = eigen_quat.x();
  quat.y = eigen_quat.y();
  quat.z = eigen_quat.z();
  return quat;
}

void FeatureExtractor::visualiseSamples()
{
  visualization_msgs::MarkerArray vis_array;

  int id = 1;
  visualization_msgs::Marker clear;
  clear.action = visualization_msgs::Marker::DELETEALL;
  vis_array.markers.push_back(clear);
  for (auto& sample : optimiser_.samples_)
  {
    visualization_msgs::Marker lidar_board, lidar_normal;

    lidar_board.header.frame_id = lidar_normal.header.frame_id = lidar_frame_;
    lidar_board.action = lidar_normal.action = visualization_msgs::Marker::ADD;
    lidar_board.type = visualization_msgs::Marker::LINE_STRIP;
    lidar_normal.type = visualization_msgs::Marker::ARROW;

    lidar_normal.scale.x = 0.5;
    lidar_normal.scale.y = 0.04;
    lidar_normal.scale.z = 0.04;
    lidar_normal.color.a = 1.0;
    lidar_normal.color.b = 1.0;
    lidar_normal.color.g = 0.0;
    lidar_normal.color.r = 0.0;
    lidar_normal.pose.position.x = sample.lidar_centre.x / 1000;
    lidar_normal.pose.position.y = sample.lidar_centre.y / 1000;
    lidar_normal.pose.position.z = sample.lidar_centre.z / 1000;
    lidar_normal.pose.orientation = normalToQuaternion(sample.lidar_normal);
    lidar_normal.id = id++;

    vis_array.markers.push_back(lidar_normal);

    lidar_board.scale.x = 0.04;
    lidar_board.scale.y = 0.04;
    lidar_board.scale.z = 0.04;
    lidar_board.color.a = 1.0;
    lidar_board.color.b = 0.0;
    lidar_board.color.g = 1.0;
    lidar_board.color.r = 0.0;
    lidar_board.pose.orientation.w = 1.0;
    for (auto& c : sample.lidar_corners)
    {
      geometry_msgs::Point p;
      p.x = c.x;
      p.y = c.y;
      p.z = c.z;
      lidar_board.points.push_back(p);
    }
    lidar_board.points.push_back(lidar_board.points[0]);
    lidar_board.id = id++;
    vis_array.markers.push_back(lidar_board);
  }
  samples_pub_.publish(vis_array);
}

void FeatureExtractor::passthrough(const PointCloud::ConstPtr& input_pc, PointCloud::Ptr& output_pc)
{
  PointCloud::Ptr x(new PointCloud);
  PointCloud::Ptr z(new PointCloud);
  // Filter out the experimental region
  pcl::PassThrough<pcl::PointXYZIR> pass;
  pass.setInputCloud(input_pc);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(bounds_.x_min, bounds_.x_max);
  pass.filter(*x);
  pass.setInputCloud(x);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(bounds_.z_min, bounds_.z_max);
  pass.filter(*z);
  pass.setInputCloud(z);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(bounds_.y_min, bounds_.y_max);
  pass.filter(*output_pc);
}

auto FeatureExtractor::chessboardProjection(const std::vector<cv::Point2d>& corners,
                                            const cv_bridge::CvImagePtr& cv_ptr)
{
  // Now find the chessboard in 3D space
  cv::Point3d chessboard_centre(i_params.chessboard_pattern_size.width, i_params.chessboard_pattern_size.height, 0);
  chessboard_centre *= 0.5 * i_params.square_length;
  std::vector<cv::Point3d> corners_3d;
  for (int y = 0; y < i_params.chessboard_pattern_size.height; y++)
  {
    for (int x = 0; x < i_params.chessboard_pattern_size.width; x++)
    {
      corners_3d.push_back(cv::Point3d(x, y, 0) * i_params.square_length - chessboard_centre);
    }
  }

  // checkerboard corners, middle square corners, board corners and centre
  std::vector<cv::Point3d> board_corners_3d;
  // Board corner coordinates from the centre of the checkerboard
  for (auto x = 0; x < 2; x++)
  {
    for (auto y = 0; y < 2; y++)
    {
      board_corners_3d.push_back(
          cv::Point3d((-0.5 + x) * i_params.board_dimensions.width, (-0.5 + y) * i_params.board_dimensions.height, 0) -
          i_params.cb_translation_error);
    }
  }
  // Board centre coordinates from the centre of the checkerboard (due to incorrect placement of checkerbord on
  // board)
  board_corners_3d.push_back(-i_params.cb_translation_error);

  std::vector<cv::Point2d> corner_image_points, board_image_points;

  cv::Mat rvec(3, 3, cv::DataType<double>::type);  // Initialization for pinhole and fisheye cameras
  cv::Mat tvec(3, 1, cv::DataType<double>::type);

  if (i_params.fisheye_model)
  {
    // Undistort the image by applying the fisheye intrinsic parameters
    // the final input param is the camera matrix in the new or rectified coordinate frame.
    // We put this to be the same as i_params.cameramat or else it will be set to empty matrix by default.
    std::vector<cv::Point2d> corners_undistorted;
    cv::fisheye::undistortPoints(corners, corners_undistorted, i_params.cameramat, i_params.distcoeff,
                                 i_params.cameramat);
    cv::solvePnP(corners_3d, corners_undistorted, i_params.cameramat, cv::noArray(), rvec, tvec);
    cv::fisheye::projectPoints(corners_3d, corner_image_points, rvec, tvec, i_params.cameramat, i_params.distcoeff);
    cv::fisheye::projectPoints(board_corners_3d, board_image_points, rvec, tvec, i_params.cameramat,
                               i_params.distcoeff);
  }
  else
  {
    // Pinhole model
    cv::solvePnP(corners_3d, corners, i_params.cameramat, i_params.distcoeff, rvec, tvec);
    cv::projectPoints(corners_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, corner_image_points);
    cv::projectPoints(board_corners_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, board_image_points);
  }
  for (auto& point : corner_image_points)
  {
    cv::circle(cv_ptr->image, point, 5, CV_RGB(255, 0, 0), -1);
  }
  for (auto& point : board_image_points)
  {
    cv::circle(cv_ptr->image, point, 5, CV_RGB(255, 255, 0), -1);
  }

  // Return all the necessary coefficients
  return std::make_tuple(rvec, tvec, board_corners_3d);
}

std::tuple<std::vector<cv::Point3d>, cv::Mat>
FeatureExtractor::locateChessboard(const sensor_msgs::Image::ConstPtr& image)
{
  // Convert to OpenCV image object
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

  cv::Mat gray;
  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
  std::vector<cv::Point2f> cornersf;
  std::vector<cv::Point2d> corners;
  // Find checkerboard pattern in the image
  bool pattern_found = findChessboardCorners(gray, i_params.chessboard_pattern_size, cornersf,
                                             cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
  if (!pattern_found)
  {
    ROS_WARN("No chessboard found");
    std::vector<cv::Point3d> empty_corners;
    cv::Mat empty_normal;
    return std::make_tuple(empty_corners, empty_normal);
  }
  ROS_INFO("Chessboard found");
  // Find corner points with sub-pixel accuracy
  // This throws an exception if the corner points are doubles and not floats!?!
  cornerSubPix(gray, cornersf, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

  for (auto& corner : cornersf)
  {
    corners.push_back(cv::Point2d(corner));
  }

  auto [rvec, tvec, board_corners_3d] = chessboardProjection(corners, cv_ptr);

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  cv::Mat z = cv::Mat(cv::Point3d(0., 0., 1.));
  auto chessboard_normal = rmat * z;

  std::vector<cv::Point3d> corner_vectors;
  for (auto& corner : board_corners_3d)
  {
    cv::Mat m(rmat * cv::Mat(corner).reshape(1) + tvec);
    corner_vectors.push_back(cv::Point3d(m));
  }

  // Publish the image with all the features marked in it
  ROS_INFO("Publishing chessboard image");
  image_publisher.publish(cv_ptr->toImageMsg());
  return std::make_tuple(corner_vectors, chessboard_normal);
}

std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>
FeatureExtractor::extractBoard(const PointCloud::Ptr& cloud)
{
  PointCloud::Ptr cloud_filtered(new PointCloud);
  // Filter out the board point cloud
  // find the point with max height(z val) in cloud_passthrough
  pcl::PointXYZIR cloud_min, cloud_max;
  pcl::getMinMax3D(*cloud, cloud_min, cloud_max);
  double z_max = cloud_max.z;
  // subtract by approximate diagonal length (in metres)
  double diag = std::hypot(i_params.board_dimensions.height, i_params.board_dimensions.width) /
                1000.0;  // board dimensions are in mm
  double z_min = z_max - diag;
  pcl::PassThrough<pcl::PointXYZIR> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_min, z_max);
  pass_z.setInputCloud(cloud);
  pass_z.filter(*cloud_filtered);  // board point cloud

  // Fit a plane through the board point cloud
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZIR> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.004);
  pcl::ExtractIndices<pcl::PointXYZIR> extract;
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);

  // Check that segmentation succeeded
  PointCloud::Ptr cloud_projected(new PointCloud);
  if (coefficients->values.size() < 3)
  {
    ROS_WARN("Checkerboard plane segmentation failed");
    cv::Point3d null_normal;
    return std::make_tuple(cloud_projected, null_normal);
  }

  // Plane normal vector magnitude
  cv::Point3d lidar_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  lidar_normal /= -cv::norm(lidar_normal);  // Normalise and flip the direction

  // Project the inliers on the fit plane
  pcl::ProjectInliers<pcl::PointXYZIR> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_filtered);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_projected);

  // Publish the projected inliers
  board_cloud_pub_.publish(cloud_projected);
  return std::make_tuple(cloud_projected, lidar_normal);
}

std::pair<pcl::ModelCoefficients, pcl::ModelCoefficients>
FeatureExtractor::findEdges(const PointCloud::Ptr& edge_pair_cloud)
{
  pcl::ModelCoefficients full_coeff, half_coeff;
  pcl::PointIndices::Ptr full_inliers(new pcl::PointIndices), half_inliers(new pcl::PointIndices);
  PointCloud::Ptr half_cloud(new PointCloud);

  pcl::SACSegmentation<pcl::PointXYZIR> seg;
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(edge_pair_cloud);
  seg.segment(*full_inliers, full_coeff);  // Fitting line1 through all points
  pcl::ExtractIndices<pcl::PointXYZIR> extract;
  extract.setInputCloud(edge_pair_cloud);
  extract.setIndices(full_inliers);
  extract.setNegative(true);
  extract.filter(*half_cloud);
  seg.setInputCloud(half_cloud);
  seg.segment(*half_inliers, half_coeff);
  // Fitting line2 through outlier points
  // Determine which is above the other
  pcl::PointXYZIR full_min, full_max, half_min, half_max;
  pcl::getMinMax3D(*edge_pair_cloud, full_min, full_max);
  pcl::getMinMax3D(*half_cloud, half_min, half_max);
  if (full_max.z > half_max.z)
  {
    return std::make_pair(full_coeff, half_coeff);
  }
  else
  {
    return std::make_pair(half_coeff, full_coeff);
  }
}

// Extract features of interest
void FeatureExtractor::extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& image,
                                               const PointCloud::ConstPtr& pointcloud)
{
  // Check if we have deduced the lidar ring count
  if (i_params.lidar_ring_count == 0)
  {
    // pcl::getMinMax3D only works on x,y,z
    for (const auto& p : pointcloud->points)
    {
      if (p.ring + 1 > i_params.lidar_ring_count)
      {
        i_params.lidar_ring_count = p.ring + 1;
      }
    }
    lidar_frame_ = pointcloud->header.frame_id;
  }
  PointCloud::Ptr cloud_bounded(new PointCloud);
  cam_lidar_calibration::OptimisationSample sample;
  passthrough(pointcloud, cloud_bounded);

  // Publish the experimental region point cloud
  bounded_cloud_pub_.publish(cloud_bounded);

  if (flag == Optimise::Request::CAPTURE)
  {
    flag = 0;  // Reset the capture flag

    ROS_INFO("Processing sample");

    auto [corner_vectors, chessboard_normal] = locateChessboard(image);
    if (corner_vectors.size() == 0)
    {
      return;
    }

    sample.camera_centre = corner_vectors[4];  // Centre of board
    corner_vectors.pop_back();
    sample.camera_corners = corner_vectors;
    sample.camera_normal = cv::Point3d(chessboard_normal);

    // FIND THE MAX AND MIN POINTS IN EVERY RING CORRESPONDING TO THE BOARD
    auto [cloud_projected, lidar_normal] = extractBoard(cloud_bounded);
    if (cloud_projected->points.size() == 0)
    {
      return;
    }
    sample.lidar_normal = lidar_normal;

    // First: Sort out the points in the point cloud according to their ring numbers
    std::vector<PointCloud> ring_pointclouds(i_params.lidar_ring_count);

    for (const auto& point : cloud_projected->points)
    {
      ring_pointclouds[point.ring].push_back(point);
    }

    // Second: Arrange points in every ring in descending order of y coordinate
    for (auto& ring : ring_pointclouds)
    {
      std::sort(ring.begin(), ring.end(), [](pcl::PointXYZIR p1, pcl::PointXYZIR p2) { return p1.y > p2.y; });
    }

    // Third: Find minimum and maximum points in a ring
    PointCloud::Ptr max_points(new PointCloud);
    PointCloud::Ptr min_points(new PointCloud);
    for (const auto& ring : ring_pointclouds)
    {
      if (ring.size() == 0)
      {
        continue;
      }
      min_points->push_back(ring[ring.size() - 1]);
      max_points->push_back(ring[0]);
    }

    // Fit lines through minimum and maximum points
    auto [top_left, bottom_left] = findEdges(max_points);
    auto [top_right, bottom_right] = findEdges(min_points);
    ROS_INFO("Found line coefficients");

    // Find the corners
    Eigen::Vector4f corner;
    pcl::lineWithLineIntersection(top_left, top_right, corner);
    cv::Point3d top(corner[0], corner[1], corner[2]);
    pcl::lineWithLineIntersection(bottom_left, bottom_right, corner);
    cv::Point3d bottom(corner[0], corner[1], corner[2]);
    pcl::lineWithLineIntersection(top_left, bottom_left, corner);
    cv::Point3d left(corner[0], corner[1], corner[2]);
    pcl::lineWithLineIntersection(top_right, bottom_right, corner);
    cv::Point3d right(corner[0], corner[1], corner[2]);
    // Add points in same order as the paper
    sample.lidar_corners.push_back(right);
    sample.lidar_corners.push_back(top);
    sample.lidar_corners.push_back(left);
    sample.lidar_corners.push_back(bottom);

    for (const auto& p : sample.lidar_corners)
    {
      // Average the corners, and convert to mm
      sample.lidar_centre.x += p.x / 4.0 * 1000.;
      sample.lidar_centre.y += p.y / 4.0 * 1000.;
      sample.lidar_centre.z += p.z / 4.0 * 1000.;
    }
    // Push this sample to the optimiser
    optimiser_.samples_.push_back(sample);
  }  // if (flag == Optimise::Request::CAPTURE)
}  // End of extractRegionOfInterest

}  // namespace cam_lidar_calibration
