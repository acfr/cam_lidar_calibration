#include "main.h"


int main(int argc, char **argv) {
  //Initialize Node and handles
  ros::init(argc, argv, "feature_extraction");
  ros::NodeHandle n;

  extrinsic_calibration::feature_extraction feature_extraction;
  feature_extraction.bypass_init();
  ros::spin();

  return 0;
}



namespace extrinsic_calibration
{

feature_extraction::feature_extraction() {}

void feature_extraction::onInit() 
{
  // Read input parameters from configuration file
  pkg_loc = ros::package::getPath("cam_lidar_calibration");
  std::ifstream infile(pkg_loc + "/cfg/initial_params.txt");

  infile >> i_params.camera_topic;
  infile >> i_params.lidar_topic;
  infile >> i_params.fisheye_model;
  infile >> i_params.lidar_ring_count;
  infile >> cb_l; infile >> cb_b; i_params.grid_size = std::make_pair(cb_l,cb_b);
  infile >> i_params.square_length;
  infile >> l; infile >> b; i_params.board_dimension = std::make_pair(l,b);
  infile >> e_l; infile >> e_b; i_params.cb_translation_error = std::make_pair(e_l,e_b);
  double camera_mat[9];
  for(int i=0; i<9; i++)
  {
    infile >> camera_mat[i];
  }
  cv::Mat(3, 3, CV_64F, &camera_mat).copyTo(i_params.cameramat);
  infile >> i_params.distcoeff_num;
  double dist_coeff[i_params.distcoeff_num];
  for(int i=0; i<i_params.distcoeff_num; i++)
  {
    infile >> dist_coeff[i];
  }
  cv::Mat(1, i_params.distcoeff_num, CV_64F, &dist_coeff).copyTo(i_params.distcoeff);
  diagonal = sqrt(pow(i_params.board_dimension.first,2) + pow(i_params.board_dimension.second,2))/1000;

  std::cout << "Input parameters received" << std::endl;

  // Creating ROS nodehandle
  ros::NodeHandle private_nh = ros::NodeHandle("~");
  ros::NodeHandle public_nh = ros::NodeHandle();
  ros::NodeHandle pnh = ros::NodeHandle("~");//getMTPrivateNodeHandle();

  it_.reset(new image_transport::ImageTransport(public_nh));
  it_p_.reset(new image_transport::ImageTransport(private_nh));
  image_sub = new image_sub_type(public_nh, i_params.camera_topic, queue_rate);
  pcl_sub = new pc_sub_type(public_nh, i_params.lidar_topic, queue_rate);

  // Dynamic reconfigure gui to set the experimental region bounds
  server = boost::make_shared <dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig> > (pnh);
  dynamic_reconfigure::Server<cam_lidar_calibration::boundsConfig>::CallbackType f;
  f = boost::bind(&feature_extraction::bounds_callback, this, _1, _2);
  server->setCallback(f);

  // Synchronizer to get synchronized camera-lidar scan pairs
  sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queue_rate), *image_sub, *pcl_sub);
  sync->registerCallback(boost::bind(&feature_extraction::extractROI, this, _1, _2));

  roi_publisher = public_nh.advertise<cam_lidar_calibration::calibration_data>("roi/points", 10, true);
  pub_cloud = public_nh.advertise<sensor_msgs::PointCloud2>("velodyne_features", 1);
  expt_region = public_nh.advertise<sensor_msgs::PointCloud2>("Experimental_region", 10);
  flag_subscriber = public_nh.subscribe<std_msgs::Int8>("/flag", 1, &feature_extraction::flag_cb, this);
  vis_pub = public_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  visPub = public_nh.advertise<visualization_msgs::Marker>( "boardcorners", 0 );
  image_publisher = it_p_->advertise("camera_features", 1);
  NODELET_INFO_STREAM("Camera Lidar Calibration");
}

void feature_extraction::flag_cb(const std_msgs::Int8::ConstPtr& msg) 
{
  flag = msg->data; // read flag published by input_sample node
}

void feature_extraction::bounds_callback(cam_lidar_calibration::boundsConfig &config, uint32_t level) 
{
  // Read the values corresponding to the motion of slider bars in reconfigure gui
  bound = config;
   ROS_INFO("Reconfigure Request: %lf %lf %lf %lf %lf %lf", config.x_min, config.x_max, config.y_min, config.y_max, config.z_min, config.z_max);
}

// Convert 3D points w.r.t camera frame to 2D pixel points in image frame
double * feature_extraction::converto_imgpts(double x, double y, double z)
{  
  double tmpxC = x/z;
  double tmpyC = y/z;
  cv::Point2d planepointsC;
  planepointsC.x = tmpxC;
  planepointsC.y = tmpyC;
  double r2 = tmpxC*tmpxC + tmpyC*tmpyC;

  if (i_params.fisheye_model)
  {
    double r1 = pow(r2,0.5);
    double a0 = std::atan(r1);
    // distortion function for a fisheye lens
    double a1 = a0*(1 + i_params.distcoeff.at<double>(0)*pow(a0,2) + i_params.distcoeff.at<double>(1)*pow(a0,4)
                    + i_params.distcoeff.at<double>(2)*pow(a0,6) + i_params.distcoeff.at<double>(3)*pow(a0,8));
    planepointsC.x = (a1/r1)*tmpxC;
    planepointsC.y = (a1/r1)*tmpyC;
    planepointsC.x = i_params.cameramat.at<double>(0,0)*planepointsC.x + i_params.cameramat.at<double>(0,2);
    planepointsC.y = i_params.cameramat.at<double>(1,1)*planepointsC.y + i_params.cameramat.at<double>(1,2);
  }
  else // For pinhole camera model
  {
    double tmpdist = 1 + i_params.distcoeff.at<double>(0)*r2 + i_params.distcoeff.at<double>(1)*r2*r2 +
        i_params.distcoeff.at<double>(4)*r2*r2*r2;
    planepointsC.x = tmpxC*tmpdist + 2*i_params.distcoeff.at<double>(2)*tmpxC*tmpyC +
        i_params.distcoeff.at<double>(3)*(r2+2*tmpxC*tmpxC);
    planepointsC.y = tmpyC*tmpdist + i_params.distcoeff.at<double>(2)*(r2+2*tmpyC*tmpyC) +
        2*i_params.distcoeff.at<double>(3)*tmpxC*tmpyC;
    planepointsC.x = i_params.cameramat.at<double>(0,0)*planepointsC.x + i_params.cameramat.at<double>(0,2);
    planepointsC.y = i_params.cameramat.at<double>(1,1)*planepointsC.y + i_params.cameramat.at<double>(1,2);
  }

  double * img_coord = new double[2];
  *(img_coord) = planepointsC.x;
  *(img_coord+1) = planepointsC.y;

  return img_coord;
}

// Extract features of interest
void feature_extraction::extractROI(const sensor_msgs::Image::ConstPtr &img, const sensor_msgs::PointCloud2::ConstPtr &pc) 
{   
  pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZIR>),
      cloud_filtered1(new pcl::PointCloud<pcl::PointXYZIR>),
      cloud_passthrough1(new pcl::PointCloud<pcl::PointXYZIR>);
  sensor_msgs::PointCloud2 cloud_final1;
  pcl::fromROSMsg (*pc, *cloud1);

  // Filter out the experimental region
  pcl::PassThrough<pcl::PointXYZIR> pass1;
  pass1.setInputCloud (cloud1);
  pass1.setFilterFieldName ("x");
  pass1.setFilterLimits (bound.x_min, bound.x_max);
  pass1.filter (*cloud_passthrough1);
  pcl::PassThrough<pcl::PointXYZIR> pass_z1;
  pass_z1.setInputCloud (cloud_passthrough1);
  pass_z1.setFilterFieldName ("z");
  pass_z1.setFilterLimits (bound.z_min, bound.z_max);
  pass_z1.filter (*cloud_passthrough1);
  pcl::PassThrough<pcl::PointXYZIR> pass_final1;
  pass_final1.setInputCloud (cloud_passthrough1);
  pass_final1.setFilterFieldName ("y");
  pass_final1.setFilterLimits (bound.y_min, bound.y_max);
  pass_final1.filter (*cloud_passthrough1);

  // Publish the experimental region point cloud
  pcl::toROSMsg(*cloud_passthrough1, cloud_final1);
  expt_region.publish(cloud_final1);

  // Runs only if user presses 'i' to get a sample
  if (flag == 1)
  {
    cv::Mat corner_vectors = cv::Mat::eye(3,5,CV_64F);
    cv::Mat chessboard_normal = cv::Mat(1,3,CV_64F);
    // checkerboard corners, middle square corners, board corners and centre
    std::vector< cv::Point2f > image_points, imagePoints1, imagePoints;
    flag = 0; // Set flag to 0

    //////////////// IMAGE FEATURES //////////////////

    cv_bridge::CvImagePtr cv_ptr;
    cv::Size2i patternNum(i_params.grid_size.first,i_params.grid_size.second);
    cv::Size2i patternSize(i_params.square_length,i_params.square_length);

    try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what()); }

    cv::Mat gray;
    std::vector<cv::Point2f> corners, corners_undistorted;
    std::vector<cv::Point3f> grid3dpoint;
    cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
    // Find checkerboard pattern in the image
    bool patternfound = cv::findChessboardCorners(gray, patternNum, corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

    if(patternfound)
    {
      // Find corner points with sub-pixel accuracy
      cornerSubPix(gray, corners, Size(11,11), Size(-1,-1),
                   TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cv::Size imgsize;
      imgsize.height = cv_ptr->image.rows;
      imgsize.width = cv_ptr->image.cols;
      double tx, ty; // Translation values
      // Location of board frame origin from the bottom left inner corner of the checkerboard
      tx = (patternNum.height - 1) * patternSize.height/2;
      ty = (patternNum.width - 1) * patternSize.width/2;
      // Board corners w.r.t board frame
      for(int i = 0; i < patternNum.height; i++)
      {
        for(int j = 0; j < patternNum.width; j++)
        {
          cv::Point3f tmpgrid3dpoint;
          // Translating origin from bottom left corner to the centre of the checkerboard
          tmpgrid3dpoint.x = i*patternSize.height - tx;
          tmpgrid3dpoint.y = j*patternSize.width - ty;
          tmpgrid3dpoint.z = 0;
          grid3dpoint.push_back(tmpgrid3dpoint);
        }
      }
      std::vector< cv::Point3f > boardcorners;
      // Board corner coordinates from the centre of the checkerboard
      boardcorners.push_back(cv::Point3f((i_params.board_dimension.second - i_params.cb_translation_error.second)/2,
                                         (i_params.board_dimension.first - i_params.cb_translation_error.first)/2, 0.0));
      boardcorners.push_back(cv::Point3f(-(i_params.board_dimension.second + i_params.cb_translation_error.second)/2,
                                         (i_params.board_dimension.first - i_params.cb_translation_error.first)/2, 0.0));
      boardcorners.push_back(cv::Point3f(-(i_params.board_dimension.second + i_params.cb_translation_error.second)/2,
                                         -(i_params.board_dimension.first + i_params.cb_translation_error.first)/2, 0.0));
      boardcorners.push_back(cv::Point3f((i_params.board_dimension.second - i_params.cb_translation_error.second)/2,
                                         -(i_params.board_dimension.first + i_params.cb_translation_error.first)/2, 0.0));
      // Board centre coordinates from the centre of the checkerboard (due to incorrect placement of checkerbord on board)
      boardcorners.push_back(cv::Point3f(-i_params.cb_translation_error.second/2,
                                         -i_params.cb_translation_error.first/2, 0.0));

      std::vector< cv::Point3f > square_edge;
      // centre checkerboard square corner coordinates wrt the centre of the checkerboard (origin)
      square_edge.push_back(cv::Point3f(-i_params.square_length/2, -i_params.square_length/2, 0.0));
      square_edge.push_back(cv::Point3f(i_params.square_length/2, i_params.square_length/2, 0.0));
      cv::Mat rvec(3,3,cv::DataType<double>::type); // Initialization for pinhole and fisheye cameras
      cv::Mat tvec(3,1,cv::DataType<double>::type);

      if (i_params.fisheye_model)
      {
        // Undistort the image by applying the fisheye intrinsic parameters
        // the final input param is the camera matrix in the new or rectified coordinate frame.
        // We put this to be the same as i_params.cameramat or else it will be set to empty matrix by default.
        cv::fisheye::undistortPoints(corners, corners_undistorted, i_params.cameramat, i_params.distcoeff,
                                     i_params.cameramat);
        cv::Mat fake_distcoeff = (Mat_<double>(4,1) << 0, 0, 0, 0);
        cv::solvePnP(grid3dpoint, corners_undistorted, i_params.cameramat, fake_distcoeff, rvec, tvec);
        cv::fisheye::projectPoints(grid3dpoint, image_points, rvec, tvec, i_params.cameramat, i_params.distcoeff);
        // Mark the centre square corner points
        cv::fisheye::projectPoints(square_edge, imagePoints1, rvec, tvec, i_params.cameramat, i_params.distcoeff);
        cv::fisheye::projectPoints(boardcorners, imagePoints, rvec, tvec, i_params.cameramat, i_params.distcoeff);
        for (int i = 0; i < grid3dpoint.size(); i++)
          cv::circle(cv_ptr->image, image_points[i], 5, CV_RGB(255,0,0), -1);

//        for (int i = 0; i < square_edge.size(); i++)
//          cv::circle(cv_ptr->image, imagePoints1[i], 5, CV_RGB(255,0,0), -1);
//        // Mark the board corner points and centre point
//        for (int i = 0; i < boardcorners.size(); i++)
//          cv::circle(cv_ptr->image, imagePoints[i], 5, CV_RGB(255,0,0), -1);

      }
      // Pinhole model
      else
      {
        cv::solvePnP(grid3dpoint, corners, i_params.cameramat, i_params.distcoeff, rvec, tvec);
        cv::projectPoints(grid3dpoint, rvec, tvec, i_params.cameramat, i_params.distcoeff, image_points);
        // Mark the centre square corner points
        cv::projectPoints(square_edge, rvec, tvec, i_params.cameramat, i_params.distcoeff, imagePoints1);
        cv::projectPoints(boardcorners, rvec, tvec, i_params.cameramat, i_params.distcoeff, imagePoints);

//        for (int i = 0; i < square_edge.size(); i++)
//          cv::circle(cv_ptr->image, imagePoints1[i], 5, CV_RGB(255,0,0), -1);
//        // Mark the board corner points and centre point
//        for (int i = 0; i < boardcorners.size(); i++)
//          cv::circle(cv_ptr->image, imagePoints[i], 5, CV_RGB(255,0,0), -1);

      }

      // chessboardpose is a 3*4 transform matrix that transforms points in board frame to camera frame | R&T
      cv::Mat chessboardpose = cv::Mat::eye(4,4,CV_64F);
      cv::Mat tmprmat = cv::Mat(3,3,CV_64F); // rotation matrix
      cv::Rodrigues(rvec,tmprmat); // Euler angles to rotation matrix

      for(int j = 0; j < 3; j++)
      {
        for(int k = 0; k < 3; k++)
        {
          chessboardpose.at<double>(j,k) = tmprmat.at<double>(j,k);
        }
        chessboardpose.at<double>(j,3) = tvec.at<double>(j);
      }

      chessboard_normal.at<double>(0) = 0;
      chessboard_normal.at<double>(1) = 0;
      chessboard_normal.at<double>(2) = 1;
      chessboard_normal = chessboard_normal * chessboardpose(cv::Rect(0,0,3,3)).t();

      for (int k = 0; k < boardcorners.size(); k++)
      {
        // take every point in boardcorners set
        cv::Point3f pt(boardcorners[k]);
        for (int i = 0; i < 3; i++)
        {
          // Transform it to obtain the coordinates in cam frame
          corner_vectors.at<double>(i,k) = chessboardpose.at<double>(i,0)*pt.x +
              chessboardpose.at<double>(i,1)*pt.y + chessboardpose.at<double>(i,3);
        }

        // convert 3D coordinates to image coordinates
        double * img_coord = feature_extraction::converto_imgpts(corner_vectors.at<double>(0,k),
                                                                 corner_vectors.at<double>(1,k),
                                                                 corner_vectors.at<double>(2,k));
        // Mark the corners and the board centre
        if (k==0)
          cv::circle(cv_ptr->image, cv::Point(img_coord[0],img_coord[1]),
              8, CV_RGB(0,255,0),-1); //green
        else if (k==1)
          cv::circle(cv_ptr->image, cv::Point(img_coord[0],img_coord[1]),
              8, CV_RGB(255,255,0),-1); //yellow
        else if (k==2)
          cv::circle(cv_ptr->image, cv::Point(img_coord[0],img_coord[1]),
              8, CV_RGB(0,0,255),-1); //blue
        else if (k==3)
          cv::circle(cv_ptr->image, cv::Point(img_coord[0],img_coord[1]),
              8, CV_RGB(255,0,0),-1); //red
        else
          cv::circle(cv_ptr->image, cv::Point(img_coord[0],img_coord[1]),
              8, CV_RGB(255,255,255),-1); //white for centre

        delete[] img_coord;
      }
      // Publish the image with all the features marked in it
      image_publisher.publish(cv_ptr->toImageMsg());
    } // if (patternfound)

    else
      ROS_ERROR("PATTERN NOT FOUND");

    //////////////// POINT CLOUD FEATURES //////////////////

    pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZIR>),
        cloud_filtered(new pcl::PointCloud<pcl::PointXYZIR>),
        cloud_passthrough(new pcl::PointCloud<pcl::PointXYZIR>),
        corrected_plane(new pcl::PointCloud<pcl::PointXYZIR>);
    sensor_msgs::PointCloud2 cloud_final;
    pcl::fromROSMsg (*pc, *cloud);
    // Filter out the experimental region
    pcl::PassThrough<pcl::PointXYZIR> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (bound.x_min, bound.x_max);
    pass.filter (*cloud_passthrough);
    pcl::PassThrough<pcl::PointXYZIR> pass_z;
    pass_z.setInputCloud (cloud_passthrough);
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (bound.z_min, bound.z_max);
    pass_z.filter (*cloud_passthrough);
    pcl::PassThrough<pcl::PointXYZIR> pass_final;
    pass_final.setInputCloud (cloud_passthrough);
    pass_final.setFilterFieldName ("y");
    pass_final.setFilterLimits (bound.y_min, bound.y_max);
    pass_final.filter (*cloud_passthrough);
    // Filter out the board point cloud
    // find the point with max height(z val) in cloud_passthrough
    double z_max = cloud_passthrough->points[0].z;
    size_t pt_index;
    for (size_t i = 0; i < cloud_passthrough->points.size(); ++i)
    {
      if (cloud_passthrough->points[i].z > z_max)
      {
        pt_index = i;
        z_max = cloud_passthrough->points[i].z;
      }
    }
    // subtract by approximate diagonal length (in metres)
    double z_min = z_max - diagonal;

    pass_z.setInputCloud (cloud_passthrough);
    pass_z.setFilterFieldName ("z");
    pass_z.setFilterLimits (z_min, z_max);
    pass_z.filter (*cloud_filtered); // board point cloud

    // Fit a plane through the board point cloud
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    int i = 0, nr_points = static_cast<int>(cloud_filtered->points.size());
    pcl::SACSegmentation<pcl::PointXYZIR> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.004);
    pcl::ExtractIndices<pcl::PointXYZIR> extract;
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    // Plane normal vector magnitude
    float mag = sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2)
        + pow(coefficients->values[2], 2));

    // Project the inliers on the fit plane
    pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::ProjectInliers<pcl::PointXYZIR> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_filtered);
    proj.setModelCoefficients (coefficients);
    proj.filter (*cloud_projected);

    // Publish the projected inliers
    pcl::toROSMsg(*cloud_filtered, cloud_final);
    pub_cloud.publish(cloud_final);

    // FIND THE MAX AND MIN POINTS IN EVERY RING CORRESPONDING TO THE BOARD

    // First: Sort out the points in the point cloud according to their ring numbers
    std::vector<std::deque<pcl::PointXYZIR*>> candidate_segments(i_params.lidar_ring_count);

    double x_projected = 0; double y_projected = 0; double z_projected = 0;
    for (size_t i = 0; i < cloud_projected->points.size(); ++i)
    {
      x_projected += cloud_projected->points[i].x;
      y_projected += cloud_projected->points[i].y;
      z_projected += cloud_projected->points[i].z;

      int ring_number = static_cast<int>(cloud_projected->points[i].ring);

      //push back the points in a particular ring number
      candidate_segments[ring_number].push_back(&(cloud_projected->points[i]));
    }

    // Second: Arrange points in every ring in descending order of y coordinate
    pcl::PointXYZIR max, min;
    pcl::PointCloud<pcl::PointXYZIR>::Ptr max_points(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr min_points(new pcl::PointCloud<pcl::PointXYZIR>);
    for (int i = 0; static_cast<size_t>(i) < candidate_segments.size(); i++)
    {
      if (candidate_segments[i].size()==0) // If no points belong to a aprticular ring number
      {
        continue;
      }
      for(int j = 0; j < candidate_segments[i].size(); j++)
      {
        for(int k = j+1; k < candidate_segments[i].size(); k++)
        {
          //If there is a larger element found on right of the point, swap
          if(candidate_segments[i][j]->y < candidate_segments[i][k]->y)
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
      if (candidate_segments[i].size()==0)
      {
        continue;
      }
      max = *candidate_segments[i][0];
      min = *candidate_segments[i][candidate_segments[i].size()-1];
      min_points->push_back(min);
      max_points->push_back(max);
    }

    // Fit lines through minimum and maximum points
    pcl::ModelCoefficients::Ptr coefficients_left_up (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_left_up (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_left_dwn (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_left_dwn (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_right_up (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_right_up (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_right_dwn (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_right_dwn (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZIR>),
        cloud_f1(new pcl::PointCloud<pcl::PointXYZIR>);

    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (max_points);
    seg.segment (*inliers_left_up, *coefficients_left_up); // Fitting line1 through max points
    extract.setInputCloud (max_points);
    extract.setIndices (inliers_left_up);
    extract.setNegative (true);
    extract.filter (*cloud_f);
    seg.setInputCloud (cloud_f);
    seg.segment (*inliers_left_dwn, *coefficients_left_dwn); // Fitting line2 through max points
    seg.setInputCloud (min_points);
    seg.segment (*inliers_right_up, *coefficients_right_up); // Fitting line1 through min points
    extract.setInputCloud (min_points);
    extract.setIndices (inliers_right_up);
    extract.setNegative (true);
    extract.filter (*cloud_f1);
    seg.setInputCloud (cloud_f1);
    seg.segment (*inliers_right_dwn, *coefficients_right_dwn); // Fitting line2 through min points

    // Find out 2 (out of the four) intersection points
    Eigen::Vector4f Point_l;
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ basic_point; // intersection points stored here
    if(pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_left_dwn, Point_l))
    {
      basic_point.x = Point_l[0];
      basic_point.y = Point_l[1];
      basic_point.z = Point_l[2];
      basic_cloud_ptr->points.push_back(basic_point);
    }
    if(pcl::lineWithLineIntersection(*coefficients_right_up, *coefficients_right_dwn, Point_l))
    {
      basic_point.x = Point_l[0];
      basic_point.y = Point_l[1];
      basic_point.z = Point_l[2];
      basic_cloud_ptr->points.push_back(basic_point);
    }

    // To determine the other 2 intersection points

    // Find out the diagonal vector(joining the line intersections from min_points and max_points)
    Eigen::Vector3f diagonal(basic_cloud_ptr->points[1].x-basic_cloud_ptr->points[0].x,
        basic_cloud_ptr->points[1].y-basic_cloud_ptr->points[0].y,
        basic_cloud_ptr->points[1].z-basic_cloud_ptr->points[0].z);
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
    if ( (diagonal.cross(p1)) .dot (diagonal.cross(p2)) > 0)
    {
      // Find out the line intersection between particular lines in min_points and max_points
      if(pcl::lineWithLineIntersection(*coefficients_left_dwn, *coefficients_right_up, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
      if(pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_right_dwn, Point_l))
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
      if(pcl::lineWithLineIntersection(*coefficients_left_dwn, *coefficients_right_dwn, Point_l))
      {
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
      if(pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_right_up, Point_l)){
        basic_point.x = Point_l[0];
        basic_point.y = Point_l[1];
        basic_point.z = Point_l[2];
        basic_cloud_ptr->points.push_back(basic_point);
      }
    }

    // input data
    sample_data.velodynepoint[0] = (basic_cloud_ptr->points[0].x + basic_cloud_ptr->points[1].x)*1000/2;
    sample_data.velodynepoint[1] = (basic_cloud_ptr->points[0].y + basic_cloud_ptr->points[1].y)*1000/2;
    sample_data.velodynepoint[2] = (basic_cloud_ptr->points[0].z + basic_cloud_ptr->points[1].z)*1000/2;
    sample_data.velodynenormal[0] = -coefficients->values[0]/mag;
    sample_data.velodynenormal[1] = -coefficients->values[1]/mag;
    sample_data.velodynenormal[2] = -coefficients->values[2]/mag;
    double top_down_radius = sqrt(pow(sample_data.velodynepoint[0]/1000,2)
        + pow(sample_data.velodynepoint[1]/1000,2));
    double x_comp = sample_data.velodynepoint[0]/1000 + sample_data.velodynenormal[0]/2;
    double y_comp = sample_data.velodynepoint[1]/1000 + sample_data.velodynenormal[1]/2;
    double vector_dist = sqrt(pow(x_comp,2) + pow(y_comp,2));
    if (vector_dist > top_down_radius)
    {
      sample_data.velodynenormal[0] = -sample_data.velodynenormal[0];
      sample_data.velodynenormal[1] = -sample_data.velodynenormal[1];
      sample_data.velodynenormal[2] = -sample_data.velodynenormal[2];
    }
    sample_data.camerapoint[0] = corner_vectors.at<double>(0,4);
    sample_data.camerapoint[1] = corner_vectors.at<double>(1,4);
    sample_data.camerapoint[2] = corner_vectors.at<double>(2,4);
    sample_data.cameranormal[0] = chessboard_normal.at<double>(0);
    sample_data.cameranormal[1] = chessboard_normal.at<double>(1);
    sample_data.cameranormal[2] = chessboard_normal.at<double>(2);
    sample_data.velodynecorner[0] = basic_cloud_ptr->points[2].x;
    sample_data.velodynecorner[1] = basic_cloud_ptr->points[2].y;
    sample_data.velodynecorner[2] = basic_cloud_ptr->points[2].z;
    sample_data.pixeldata = sqrt(pow((imagePoints1[1].x - imagePoints1[0].x), 2) +
        pow((imagePoints1[1].y - imagePoints1[0].y),2));

    // Visualize 4 corner points of velodyne board, the board edge lines and the centre point
    visualization_msgs::Marker marker1, line_strip, corners_board;
    marker1.header.frame_id = line_strip.header.frame_id = corners_board.header.frame_id = "/velodyne_front_link";
    marker1.header.stamp = line_strip.header.stamp = corners_board.header.stamp = ros::Time();
    marker1.ns = line_strip.ns = corners_board.ns = "my_sphere";
    line_strip.id = 10; marker1.id = 11;
    marker1.type = visualization_msgs::Marker::POINTS; line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    corners_board.type = visualization_msgs::Marker::SPHERE;
    marker1.action = line_strip.action = corners_board.action = visualization_msgs::Marker::ADD;
    marker1.pose.orientation.w = line_strip.pose.orientation.w = corners_board.pose.orientation.w = 1.0;
    marker1.scale.x = 0.02; marker1.scale.y = 0.02;
    corners_board.scale.x = 0.04; corners_board.scale.y = 0.04; corners_board.scale.z = 0.04;
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
        corners_board.pose.position.x = sample_data.velodynepoint[0]/1000;
        corners_board.pose.position.y = sample_data.velodynepoint[1]/1000;
        corners_board.pose.position.z = sample_data.velodynepoint[2]/1000;
      }

      corners_board.id = i;
      if (corners_board.id == 0)
        corners_board.color.b = 1.0;
      else if (corners_board.id == 1) {
        corners_board.color.b = 0.0; corners_board.color.g = 1.0; }
      else if (corners_board.id == 2) {
        corners_board.color.b = 0.0; corners_board.color.g = 0.0; corners_board.color.r = 1.0; }
      else if (corners_board.id == 3) {
        corners_board.color.b = 0.0; corners_board.color.r = 1.0; corners_board.color.g = 1.0; }
      else if (corners_board.id == 4) {
        corners_board.color.b = 1.0; corners_board.color.r = 1.0; corners_board.color.g = 1.0; }
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
    minmax.color.a = 1.0; // Don't forget to set the alpha!
    int y_min_pts;
    for (y_min_pts = 0; y_min_pts < min_points->points.size(); y_min_pts++)
    {
      minmax.id = y_min_pts + 13;
      minmax.pose.position.x = min_points->points[y_min_pts].x;
      minmax.pose.position.y = min_points->points[y_min_pts].y;
      minmax.pose.position.z = min_points->points[y_min_pts].z;
      minmax.color.b = 1.0; minmax.color.r = 1.0; minmax.color.g = 0.0;
      visPub.publish(minmax);
    }
    for (int y_max_pts = 0; y_max_pts < max_points->points.size(); y_max_pts++)
    {
      minmax.id = y_min_pts + 13 + y_max_pts;
      minmax.pose.position.x = max_points->points[y_max_pts].x;
      minmax.pose.position.y = max_points->points[y_max_pts].y;
      minmax.pose.position.z = max_points->points[y_max_pts].z;
      minmax.color.r = 0.0; minmax.color.g = 1.0; minmax.color.b = 1.0;
      visPub.publish(minmax);
    }
    // Draw board edge lines
    for (int i = 0; i < 2; i++)
    {
      geometry_msgs::Point p;
      p.x = basic_cloud_ptr->points[1-i].x;
      p.y = basic_cloud_ptr->points[1-i].y;
      p.z = basic_cloud_ptr->points[1-i].z;
      marker1.points.push_back(p);
      line_strip.points.push_back(p);
      p.x = basic_cloud_ptr->points[3-i].x;
      p.y = basic_cloud_ptr->points[3-i].y;
      p.z = basic_cloud_ptr->points[3-i].z;
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
    start.x = sample_data.velodynepoint[0]/1000;
    start.y = sample_data.velodynepoint[1]/1000;
    start.z = sample_data.velodynepoint[2]/1000;
    end.x = start.x + sample_data.velodynenormal[0]/2;
    end.y = start.y + sample_data.velodynenormal[1]/2;
    end.z = start.z + sample_data.velodynenormal[2]/2;
    marker.points.resize(2);
    marker.points[0].x = start.x;
    marker.points[0].y = start.y;
    marker.points[0].z = start.z;
    marker.points[1].x = end.x;
    marker.points[1].y = end.y;
    marker.points[1].z = end.z;
    // Publish Board normal
    vis_pub.publish(marker);

  } // if (flag == 1)

  // Feature data is published(chosen) only if 'enter' is pressed
  if (flag == 4)
  {
    roi_publisher.publish(sample_data);
    flag = 0;
  }

} //End of extractROI

} //End of namespace
