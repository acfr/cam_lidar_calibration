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
  for (auto& sample : optimiser_.samples_)
  {
    visualization_msgs::Marker clear, lidar_board, lidar_normal;

    clear.action = visualization_msgs::Marker::DELETEALL;
    vis_array.markers.push_back(clear);

    lidar_board.header.frame_id = lidar_normal.header.frame_id = lidar_frame_;
    lidar_board.action = lidar_normal.action = visualization_msgs::Marker::ADD;
    lidar_board.type = visualization_msgs::Marker::SPHERE;
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
      lidar_board.pose.position.x = c.x;
      lidar_board.pose.position.y = c.y;
      lidar_board.pose.position.z = c.z;
      lidar_board.id = id++;
      vis_array.markers.push_back(lidar_board);
    }
  }
  samples_pub_.publish(vis_array);

  // Visualize 4 corner points of velodyne board, the board edge lines and the centre point
  /*
  marker1.header.frame_id = line_strip.header.frame_id = corners_board.header.frame_id = "/velodyne_front_link";
  marker1.header.stamp = line_strip.header.stamp = corners_board.header.stamp = ros::Time();
  marker1.ns = line_strip.ns = corners_board.ns = "my_sphere";
  line_strip.id = 10;
  marker1.id = 11;
  marker1.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  corners_board.type = visualization_msgs::Marker::SPHERE;
  marker1.action = line_strip.action = corners_board.action = visualization_msgs::Marker::ADD;
  marker1.pose.orientation.w = line_strip.pose.orientation.w = corners_board.pose.orientation.w = 1.0;
  marker1.scale.x = 0.02;
  marker1.scale.y = 0.02;
  corners_board.scale.x = 0.04;
  corners_board.scale.y = 0.04;
  corners_board.scale.z = 0.04;
  line_strip.scale.x = 0.009;
  marker1.color.a = line_strip.color.a = corners_board.color.a = 1.0;
  line_strip.color.b = 1.0;
  marker1.color.b = marker1.color.g = marker1.color.r = 1.0;

  for (int i = 0; i < 5; i++)
  {
    if (i < 4)
    {
      corners_board.pose.position.x = basic_cloud_ptr->points[i].x;
      corners_board.pose.position.y = basic_cloud_ptr->points[i].y;
      corners_board.pose.position.z = basic_cloud_ptr->points[i].z;
    }
    else
    {
      corners_board.pose.position.x = sample.lidar_centre.x / 1000;
      corners_board.pose.position.y = sample.lidar_centre.y / 1000;
      corners_board.pose.position.z = sample.lidar_centre.z / 1000;
    }

    corners_board.id = i;
    if (corners_board.id == 0)
      corners_board.color.b = 1.0;
    else if (corners_board.id == 1)
    {
      corners_board.color.b = 0.0;
      corners_board.color.g = 1.0;
    }
    else if (corners_board.id == 2)
    {
      corners_board.color.b = 0.0;
      corners_board.color.g = 0.0;
      corners_board.color.r = 1.0;
    }
    else if (corners_board.id == 3)
    {
      corners_board.color.b = 0.0;
      corners_board.color.r = 1.0;
      corners_board.color.g = 1.0;
    }
    else if (corners_board.id == 4)
    {
      corners_board.color.b = 1.0;
      corners_board.color.r = 1.0;
      corners_board.color.g = 1.0;
    }
    visPub.publish(corners_board);
  }

  // Visualize minimum and maximum points
  visualization_msgs::Marker minmax;
  minmax.header.frame_id = "/velodyne_front_link";
  minmax.header.stamp = ros::Time();
  minmax.ns = "my_sphere";
  minmax.type = visualization_msgs::Marker::SPHERE;
  minmax.action = visualization_msgs::Marker::ADD;
  minmax.pose.orientation.w = 1.0;
  minmax.scale.x = 0.02;
  minmax.scale.y = 0.02;
  minmax.scale.z = 0.02;
  minmax.color.a = 1.0;  // Don't forget to set the alpha!
  size_t y_min_pts = 0;
  for (y_min_pts = 0; y_min_pts < min_points->points.size(); y_min_pts++)
  {
    minmax.id = y_min_pts + 13;
    minmax.pose.position.x = min_points->points[y_min_pts].x;
    minmax.pose.position.y = min_points->points[y_min_pts].y;
    minmax.pose.position.z = min_points->points[y_min_pts].z;
    minmax.color.b = 1.0;
    minmax.color.r = 1.0;
    minmax.color.g = 0.0;
    visPub.publish(minmax);
  }
  for (size_t y_max_pts = 0; y_max_pts < max_points->points.size(); y_max_pts++)
  {
    minmax.id = y_min_pts + 13 + y_max_pts;
    minmax.pose.position.x = max_points->points[y_max_pts].x;
    minmax.pose.position.y = max_points->points[y_max_pts].y;
    minmax.pose.position.z = max_points->points[y_max_pts].z;
    minmax.color.r = 0.0;
    minmax.color.g = 1.0;
    minmax.color.b = 1.0;
    visPub.publish(minmax);
  }
  // Draw board edge lines
  for (int i = 0; i < 2; i++)
  {
    geometry_msgs::Point p;
    p.x = basic_cloud_ptr->points[1 - i].x;
    p.y = basic_cloud_ptr->points[1 - i].y;
    p.z = basic_cloud_ptr->points[1 - i].z;
    marker1.points.push_back(p);
    line_strip.points.push_back(p);
    p.x = basic_cloud_ptr->points[3 - i].x;
    p.y = basic_cloud_ptr->points[3 - i].y;
    p.z = basic_cloud_ptr->points[3 - i].z;
    marker1.points.push_back(p);
    line_strip.points.push_back(p);
  }

  geometry_msgs::Point p;
  p.x = basic_cloud_ptr->points[1].x;
  p.y = basic_cloud_ptr->points[1].y;
  p.z = basic_cloud_ptr->points[1].z;
  marker1.points.push_back(p);
  line_strip.points.push_back(p);
  p.x = basic_cloud_ptr->points[0].x;
  p.y = basic_cloud_ptr->points[0].y;
  p.z = basic_cloud_ptr->points[0].z;
  marker1.points.push_back(p);
  line_strip.points.push_back(p);

  // Publish board edges
  visPub.publish(line_strip);

  // Visualize board normal vector
  marker.header.frame_id = "/velodyne_front_link";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 12;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;
  marker.scale.y = 0.04;
  marker.scale.z = 0.06;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  geometry_msgs::Point start, end;
  start.x = sample.lidar_centre.x / 1000;
  start.y = sample.lidar_centre.y / 1000;
  start.z = sample.lidar_centre.z / 1000;
  end.x = start.x + sample.lidar_normal.x / 2;
  end.y = start.y + sample.lidar_normal.y / 2;
  end.z = start.z + sample.lidar_normal.z / 2;
  marker.points.resize(2);
  marker.points[0].x = start.x;
  marker.points[0].y = start.y;
  marker.points[0].z = start.z;
  marker.points[1].x = end.x;
  marker.points[1].y = end.y;
  marker.points[1].z = end.z;
  // Publish Board normal
  samples_pub_.publish(marker);
  */
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

std::optional<std::tuple<std::vector<cv::Point3d>, cv::Mat>>
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
    return std::nullopt;
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
  cv::Mat z{ 0., 0., 1. };
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
  return std::make_optional(std::make_tuple(corner_vectors, chessboard_normal));
}

std::optional<std::tuple<pcl::PointCloud<pcl::PointXYZIR>::Ptr, cv::Point3d>>
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
  if (coefficients->values.size() < 3)
  {
    ROS_WARN("Checkerboard plane segmentation failed");
    return std::nullopt;
  }

  // Plane normal vector magnitude
  cv::Point3d lidar_normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
  lidar_normal /= -cv::norm(lidar_normal);  // Normalise and flip the direction

  // Project the inliers on the fit plane
  PointCloud::Ptr cloud_projected(new PointCloud);
  pcl::ProjectInliers<pcl::PointXYZIR> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_filtered);
  proj.setModelCoefficients(coefficients);
  proj.filter(*cloud_projected);

  // Publish the projected inliers
  board_cloud_pub_.publish(cloud_projected);
  return std::make_optional(std::make_tuple(cloud_projected, lidar_normal));
}

// Extract features of interest
void FeatureExtractor::extractRegionOfInterest(const sensor_msgs::Image::ConstPtr& image,
                                               const PointCloud::ConstPtr& pointcloud)
{
  // Check if we have deduced the lidar ring count
  if (i_params.lidar_ring_count == 0)
  {
    pcl::PointXYZIR min, max;
    pcl::getMinMax3D(*pointcloud, min, max);
    i_params.lidar_ring_count = max.ring + 1;
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

    auto chessboard_img = locateChessboard(image);
    if (!chessboard_img)
    {
      return;
    }
    auto [corner_vectors, chessboard_normal] = *chessboard_img;

    sample.camera_centre = corner_vectors[4];  // Centre of board
    sample.camera_normal = cv::Point3d(chessboard_normal);

    //////////////// POINT CLOUD FEATURES //////////////////

    // FIND THE MAX AND MIN POINTS IN EVERY RING CORRESPONDING TO THE BOARD
    auto chessboard_lidar = extractBoard(cloud_bounded);
    if (!chessboard_lidar)
    {
      return;
    }
    auto [cloud_projected, lidar_normal] = *chessboard_lidar;
    sample.lidar_normal = lidar_normal;

    // First: Sort out the points in the point cloud according to their ring numbers
    std::vector<std::deque<pcl::PointXYZIR*>> candidate_segments(i_params.lidar_ring_count);

    double x_projected = 0;
    double y_projected = 0;
    double z_projected = 0;
    for (auto& point : cloud_projected->points)
    {
      x_projected += point.x;
      y_projected += point.y;
      z_projected += point.z;

      int ring_number = static_cast<int>(point.ring);

      // push back the points in a particular ring number
      candidate_segments[ring_number].push_back(&(point));
    }

    // Second: Arrange points in every ring in descending order of y coordinate
    pcl::PointXYZIR max, min;
    PointCloud::Ptr max_points(new PointCloud);
    PointCloud::Ptr min_points(new PointCloud);
    for (int i = 0; static_cast<size_t>(i) < candidate_segments.size(); i++)
    {
      if (candidate_segments[i].size() == 0)  // If no points belong to a aprticular ring number
      {
        continue;
      }
      for (size_t j = 0; j < candidate_segments[i].size(); j++)
      {
        for (size_t k = j + 1; k < candidate_segments[i].size(); k++)
        {
          // If there is a larger element found on right of the point, swap
          if (candidate_segments[i][j]->y < candidate_segments[i][k]->y)
          {
            pcl::PointXYZIR temp;
            temp = *candidate_segments[i][k];
            *candidate_segments[i][k] = *candidate_segments[i][j];
            *candidate_segments[i][j] = temp;
          }
        }
      }
    }

    // Third: Find minimum and maximum points in a ring
    for (int i = 0; static_cast<size_t>(i) < candidate_segments.size(); i++)
    {
      if (candidate_segments[i].size() == 0)
      {
        continue;
      }
      max = *candidate_segments[i][0];
      min = *candidate_segments[i][candidate_segments[i].size() - 1];
      min_points->push_back(min);
      max_points->push_back(max);
    }

    // Fit lines through minimum and maximum points
    pcl::ModelCoefficients::Ptr coefficients_left_up(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_left_up(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_left_dwn(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_left_dwn(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_right_up(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_right_up(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_right_dwn(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_right_dwn(new pcl::PointIndices);
    PointCloud::Ptr cloud_f(new PointCloud), cloud_f1(new PointCloud);

    pcl::SACSegmentation<pcl::PointXYZIR> seg;
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(max_points);
    seg.segment(*inliers_left_up, *coefficients_left_up);  // Fitting line1 through max points
    pcl::ExtractIndices<pcl::PointXYZIR> extract;
    extract.setInputCloud(max_points);
    extract.setIndices(inliers_left_up);
    extract.setNegative(true);
    extract.filter(*cloud_f);
    seg.setInputCloud(cloud_f);
    seg.segment(*inliers_left_dwn, *coefficients_left_dwn);  // Fitting line2 through max points
    seg.setInputCloud(min_points);
    seg.segment(*inliers_right_up, *coefficients_right_up);  // Fitting line1 through min points
    extract.setInputCloud(min_points);
    extract.setIndices(inliers_right_up);
    extract.setNegative(true);
    extract.filter(*cloud_f1);
    seg.setInputCloud(cloud_f1);
    seg.segment(*inliers_right_dwn, *coefficients_right_dwn);  // Fitting line2 through min points

    // Find out 2 (out of the four) intersection points
    Eigen::Vector4f Point_l;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ basic_point;  // intersection points stored here
    if (pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_left_dwn, Point_l))
    {
      basic_point.x = Point_l[0];
      basic_point.y = Point_l[1];
      basic_point.z = Point_l[2];
      basic_cloud_ptr->points.push_back(basic_point);
    }
    if (pcl::lineWithLineIntersection(*coefficients_right_up, *coefficients_right_dwn, Point_l))
    {
      basic_point.x = Point_l[0];
      basic_point.y = Point_l[1];
      basic_point.z = Point_l[2];
      basic_cloud_ptr->points.push_back(basic_point);
    }

    // To determine the other 2 intersection points

    // Find out the diagonal vector(joining the line intersections from min_points and max_points)
    Eigen::Vector3f diagonal(basic_cloud_ptr->points[1].x - basic_cloud_ptr->points[0].x,
                             basic_cloud_ptr->points[1].y - basic_cloud_ptr->points[0].y,
                             basic_cloud_ptr->points[1].z - basic_cloud_ptr->points[0].z);
    // Take the first points in min_points and max_points,
    // find the vectors joining them and the (intersection point of the two lines in min_points)
    Eigen::Vector3f p1(min_points->points[inliers_right_dwn->indices[0]].x - basic_cloud_ptr->points[0].x,
                       min_points->points[inliers_right_dwn->indices[0]].y - basic_cloud_ptr->points[0].y,
                       min_points->points[inliers_right_dwn->indices[0]].z - basic_cloud_ptr->points[0].z);
    Eigen::Vector3f p2(max_points->points[inliers_left_dwn->indices[0]].x - basic_cloud_ptr->points[0].x,
                       max_points->points[inliers_left_dwn->indices[0]].y - basic_cloud_ptr->points[0].y,
                       max_points->points[inliers_left_dwn->indices[0]].z - basic_cloud_ptr->points[0].z);
    // To check if p1 and p2 lie on the same side or opp. side of diagonal vector.
    // If they lie on the same side
    if ((diagonal.cross(p1)).dot(diagonal.cross(p2)) > 0)
    {
      // Find out the line intersection between particular lines in min_points and max_points
      if (pcl::lineWithLineIntersection(*coefficients_left_dwn, *coefficients_right_up, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
      if (pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_right_dwn, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
    }
    // Else if they lie on the opp. side
    else
    {
      // Find out the line intersection between other lines in min_points and max_points
      if (pcl::lineWithLineIntersection(*coefficients_left_dwn, *coefficients_right_dwn, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
      if (pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_right_up, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
    }

    // input data
    sample.lidar_centre.x = (basic_cloud_ptr->points[0].x + basic_cloud_ptr->points[1].x) * 1000 / 2;
    sample.lidar_centre.y = (basic_cloud_ptr->points[0].y + basic_cloud_ptr->points[1].y) * 1000 / 2;
    sample.lidar_centre.z = (basic_cloud_ptr->points[0].z + basic_cloud_ptr->points[1].z) * 1000 / 2;
    double top_down_radius = sqrt(pow(sample.lidar_centre.x / 1000, 2) + pow(sample.lidar_centre.y / 1000, 2));
    double x_comp = sample.lidar_centre.x / 1000 + sample.lidar_normal.x / 2;
    double y_comp = sample.lidar_centre.y / 1000 + sample.lidar_normal.y / 2;
    double vector_dist = sqrt(pow(x_comp, 2) + pow(y_comp, 2));
    if (vector_dist > top_down_radius)
    {
      sample.lidar_normal = -sample.lidar_normal;
    }
    sample.lidar_corner.x = basic_cloud_ptr->points[2].x;
    sample.lidar_corner.y = basic_cloud_ptr->points[2].y;
    sample.lidar_corner.z = basic_cloud_ptr->points[2].z;

    for (auto& pt : basic_cloud_ptr->points)
    {
      cv::Point3d p(pt.x, pt.y, pt.z);
      sample.lidar_corners.push_back(p);
    }

    // Push this sample to the optimiser
    optimiser_.samples_.push_back(sample);
  }  // if (flag == Optimise::Request::CAPTURE)
}  // End of extractRegionOfInterest

}  // namespace cam_lidar_calibration
