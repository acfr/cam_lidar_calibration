#include "cam_lidar_calibration/optimiser.h"

#include "cam_lidar_calibration/point_xyzir.h"

#include <tf/transform_datatypes.h>

namespace cam_lidar_calibration
{
cv::Mat operator*(const Rotation& lhs, const cv::Point3d& rhs)
{
  using cv::Mat_;

  cv::Mat mat = cv::Mat(rhs).reshape(1);
  // Combined rotation matrix
  return lhs.toMat() * mat;
}
cv::Mat operator*(const RotationTranslation& lhs, const cv::Point3d& rhs)
{
  auto rotated = cv::Point3d(lhs.rot * rhs);
  rotated.x += lhs.x;
  rotated.y += lhs.y;
  rotated.z += lhs.z;
  return cv::Mat(rotated).reshape(1);
}

void Optimiser::init_genes2(RotationTranslation& p, const std::function<double(void)>& rnd01)
{
  std::vector<double> pi_vals;
  pi_vals.push_back(M_PI / 18);
  pi_vals.push_back(-M_PI / 18);
  int RandIndex = rand() % 2;
  p.rot.roll = eul_t.rot.roll + pi_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.rot.pitch = eul_t.rot.pitch + pi_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.rot.yaw = eul_t.rot.yaw + pi_vals.at(RandIndex) * rnd01();

  std::vector<double> trans_vals;
  trans_vals.push_back(0.05);
  trans_vals.push_back(-0.05);
  RandIndex = rand() % 2;
  p.x = eul_t.x + trans_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.y = eul_t.y + trans_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.z = eul_t.z + trans_vals.at(RandIndex) * rnd01();
}

double Optimiser::perpendicularCost(const Rotation& rot)
{
  // TODO - At the moment this is reversed from the paper.
  // We are currently calculating everything relative to the lidar frame
  // The paper does everything in the camera frame
  // Eq (3) in the paper
  double cost = 0;
  for (const auto& sample : samples_)
  {
    auto camera_normal_lidar_frame = rot * sample.camera_normal;
    auto perp = cv::Mat(sample.lidar_centre - sample.lidar_corners[0]).reshape(1);
    perp /= cv::norm(perp);
    cost += std::pow(perp.dot(camera_normal_lidar_frame), 2) / samples_.size();
  }
  return cost;
}

double Optimiser::normalAlignmentCost(const Rotation& rot)
{
  // Eq (4)
  // TODO - At the moment this is reversed from the paper.
  // We are currently calculating everything relative to the lidar frame
  // The paper does everything in the camera frame
  double cost = 0;
  for (const auto& sample : samples_)
  {
    auto camera_normal_lidar_frame = rot * sample.camera_normal;
    cost += cv::norm(camera_normal_lidar_frame - cv::Mat(sample.lidar_normal).reshape(1)) / samples_.size();
  }
  return cost;
}

double Optimiser::reprojectionCost(const RotationTranslation& rot_trans)
{
  // TODO - this is currently operating in the lidar frame
  // The paper uses the camera frame
  cv::Mat rvec = cv::Mat_<double>::zeros(3, 1);
  cv::Mat tvec = cv::Mat_<double>::zeros(3, 1);

  double cost = 0;
  for (auto& sample : samples_)
  {
    std::vector<cv::Point3d> cam_centre_3d;
    std::vector<cv::Point3d> lidar_centre_3d;

    auto camera_centre_lidar_frame = cv::Point3d(rot_trans * sample.camera_centre);
    cam_centre_3d.push_back(camera_centre_lidar_frame);
    lidar_centre_3d.push_back(sample.lidar_centre);

    std::vector<cv::Point2d> cam, lidar;
    if (i_params.fisheye_model)
    {
      cv::fisheye::projectPoints(cam_centre_3d, cam, rvec, tvec, i_params.cameramat, i_params.distcoeff);
      cv::fisheye::projectPoints(lidar_centre_3d, lidar, rvec, tvec, i_params.cameramat, i_params.distcoeff);
    }
    else
    {
      cv::projectPoints(cam_centre_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, cam);
      cv::projectPoints(lidar_centre_3d, rvec, tvec, i_params.cameramat, i_params.distcoeff, lidar);
    }
    double error = cv::norm(cam[0] - lidar[0]);
    if (error > cost)
    {
      cost = error;
    }
  }
  return cost;
}

double Optimiser::centreAlignmentCost(const RotationTranslation& rot_trans)
{
  // Eq (6) and (7)
  // TODO - At the moment this is reversed from the paper.
  // We are currently calculating everything relative to the lidar frame
  // The paper does everything in the camera frame
  double abs_mean = 0;
  for (const auto& sample : samples_)
  {
    auto camera_centre_lidar_frame = rot_trans * sample.camera_centre;
    abs_mean += cv::norm(camera_centre_lidar_frame - cv::Mat(sample.lidar_centre).reshape(1)) / samples_.size();
  }
  double stddev = 0;
  for (const auto& sample : samples_)
  {
    auto camera_centre_lidar_frame = rot_trans * sample.camera_centre;
    stddev += std::pow(cv::norm(camera_centre_lidar_frame - cv::Mat(sample.lidar_centre).reshape(1)) - abs_mean, 2) /
              samples_.size();
  }
  stddev = std::sqrt(stddev);
  return abs_mean + stddev;
}

bool Optimiser::eval_solution2(const RotationTranslation& p, RotationTranslationCost& c)
{
  double perpendicular_cost = perpendicularCost(p.rot);
  double normal_align_cost = normalAlignmentCost(p.rot);

  double centre_align_cost = centreAlignmentCost(p);
  double repro_cost = reprojectionCost(p);
  c.objective2 = perpendicular_cost + normal_align_cost + centre_align_cost + repro_cost;

  return true;  // solution is accepted
}

RotationTranslation Optimiser::mutate2(const RotationTranslation& X_base, const std::function<double(void)>& rnd01,
                                       double shrink_scale)
{
  RotationTranslation X_new;
  bool in_range;
  do
  {
    in_range = true;
    X_new = X_base;
    X_new.rot.roll += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range =
        in_range && (X_new.rot.roll >= (eul_t.rot.roll - M_PI / 18) && X_new.rot.roll < (eul_t.rot.roll + M_PI / 18));
    X_new.rot.pitch += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range &&
               (X_new.rot.pitch >= (eul_t.rot.pitch - M_PI / 18) && X_new.rot.pitch < (eul_t.rot.pitch + M_PI / 18));
    X_new.rot.yaw += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range =
        in_range && (X_new.rot.yaw >= (eul_t.rot.yaw - M_PI / 18) && X_new.rot.yaw < (eul_t.rot.yaw + M_PI / 18));

    X_new.x += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.x >= (eul_t.x - 0.05) && X_new.x < (eul_t.x + 0.05));
    X_new.y += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.y >= (eul_t.y - 0.05) && X_new.y < (eul_t.y + 0.05));
    X_new.z += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.z >= (eul_t.z - 0.05) && X_new.z < (eul_t.z + 0.05));

  } while (!in_range);
  return X_new;
}

RotationTranslation Optimiser::crossover2(const RotationTranslation& X1, const RotationTranslation& X2,
                                          const std::function<double(void)>& rnd01)
{
  RotationTranslation X_new;
  double r;
  r = rnd01();
  X_new.rot.roll = r * X1.rot.roll + (1.0 - r) * X2.rot.roll;
  r = rnd01();
  X_new.rot.pitch = r * X1.rot.pitch + (1.0 - r) * X2.rot.pitch;
  r = rnd01();
  X_new.rot.yaw = r * X1.rot.yaw + (1.0 - r) * X2.rot.yaw;
  r = rnd01();
  X_new.x = r * X1.x + (1.0 - r) * X2.x;
  r = rnd01();
  X_new.y = r * X1.y + (1.0 - r) * X2.y;
  r = rnd01();
  X_new.z = r * X1.z + (1.0 - r) * X2.z;
  return X_new;
}

double Optimiser::calculate_SO_total_fitness2(const GA_Type2::thisChromosomeType& X)
{
  // finalize the cost
  double final_cost = 0.0;
  final_cost += X.middle_costs.objective2;
  return final_cost;
}

// A function to show/store the results of each generation.
void Optimiser::SO_report_generation2(
    int generation_number, const EA::GenerationType<RotationTranslation, RotationTranslationCost>& last_generation,
    const RotationTranslation& best_genes)
{
  eul_it.rot.roll = best_genes.rot.roll;
  eul_it.rot.pitch = best_genes.rot.pitch;
  eul_it.rot.yaw = best_genes.rot.yaw;
  eul_it.x = best_genes.x;
  eul_it.y = best_genes.y;
  eul_it.z = best_genes.z;
}

void Optimiser::init_genes(Rotation& p, const std::function<double(void)>& rnd01)
{
  std::vector<double> pi_vals;
  pi_vals.push_back(M_PI / 8);
  pi_vals.push_back(-M_PI / 8);
  int RandIndex = rand() % 2;
  p.roll = eul.roll + pi_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.pitch = eul.pitch + pi_vals.at(RandIndex) * rnd01();
  RandIndex = rand() % 2;
  p.yaw = eul.yaw + pi_vals.at(RandIndex) * rnd01();
}

bool Optimiser::eval_solution(const Rotation& p, RotationCost& c)
{
  c.objective1 = perpendicularCost(p) + normalAlignmentCost(p);

  return true;  // solution is accepted
}

Rotation Optimiser::mutate(const Rotation& X_base, const std::function<double(void)>& rnd01, double shrink_scale)
{
  Rotation X_new;
  bool in_range;
  do
  {
    in_range = true;
    X_new = X_base;
    X_new.roll += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.roll >= (eul.roll - M_PI / 8) && X_new.roll < (eul.roll + M_PI / 8));
    X_new.pitch += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.pitch >= (eul.pitch - M_PI / 8) && X_new.pitch < (eul.pitch + M_PI / 8));
    X_new.yaw += 0.2 * (rnd01() - rnd01()) * shrink_scale;
    in_range = in_range && (X_new.yaw >= (eul.yaw - M_PI / 8) && X_new.yaw < (eul.yaw + M_PI / 8));
  } while (!in_range);
  return X_new;
}

Rotation Optimiser::crossover(const Rotation& X1, const Rotation& X2, const std::function<double(void)>& rnd01)
{
  Rotation X_new;
  double r = rnd01();
  X_new.roll = r * X1.roll + (1.0 - r) * X2.roll;
  r = rnd01();
  X_new.pitch = r * X1.pitch + (1.0 - r) * X2.pitch;
  r = rnd01();
  X_new.yaw = r * X1.yaw + (1.0 - r) * X2.yaw;
  return X_new;
}

double Optimiser::calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X)
{
  double final_cost = 0.0;  // finalize the cost
  final_cost += X.middle_costs.objective1;
  return final_cost;
}

// A function to show/store the results of each generation.
void Optimiser::SO_report_generation(int generation_number,
                                     const EA::GenerationType<Rotation, RotationCost>& last_generation,
                                     const Rotation& best_genes)
{
  eul_t.rot.roll = best_genes.roll;
  eul_t.rot.pitch = best_genes.pitch;
  eul_t.rot.yaw = best_genes.yaw;
}

bool Optimiser::optimise()
{
  auto n = samples_.size();
  ROS_INFO_STREAM("Optimising " << n << " collected samples");
  for (auto& sample : samples_)
  {
    ROS_INFO_STREAM("Sample");
    ROS_INFO_STREAM("Camera normal:" << sample.camera_normal);
    ROS_INFO_STREAM("Camera centre:" << sample.camera_centre);
    for (auto& c : sample.camera_corners)
    {
      ROS_INFO_STREAM("Camera corner:" << c);
    }
    ROS_INFO_STREAM("Lidar normal:" << sample.lidar_normal);
    ROS_INFO_STREAM("Lidar centre:" << sample.lidar_centre);
    for (auto& c : sample.lidar_corners)
    {
      ROS_INFO_STREAM("Lidar corner:" << c);
    }
  }
  auto camera_normals_ = cv::Mat(n, 3, CV_64F);
  auto camera_centres_ = cv::Mat(n, 3, CV_64F);
  auto lidar_centres_ = cv::Mat(n, 3, CV_64F);
  auto lidar_normals_ = cv::Mat(n, 3, CV_64F);

  // Insert vector elements into cv::Mat for easy matrix operation
  int row = 0;
  for (auto& sample : samples_)
  {
    cv::Mat cn = cv::Mat(sample.camera_normal).reshape(1).t();
    cn.copyTo(camera_normals_.row(row));
    cv::Mat cc = cv::Mat(sample.camera_centre).reshape(1).t();
    cc.copyTo(camera_centres_.row(row));
    cv::Mat ln = cv::Mat(sample.lidar_normal).reshape(1).t();
    ln.copyTo(lidar_normals_.row(row));
    cv::Mat lc = cv::Mat(sample.lidar_centre).reshape(1).t();
    lc.copyTo(lidar_centres_.row(row));
    row++;
  }
  cv::Mat NN = camera_normals_.t() * camera_normals_;
  cv::Mat NM = camera_normals_.t() * lidar_normals_;
  cv::Mat UNR = (NN.inv() * NM).t();  // Analytical rotation matrix for real data
  ROS_INFO_STREAM("Analytical rotation matrix " << UNR);
  std::vector<double> euler;
  euler = rotm2eul(UNR);  // rpy wrt original axes
  ROS_INFO_STREAM("Analytical Euler angles " << euler.at(0) << " " << euler.at(1) << " " << euler.at(2));
  eul.roll = euler[0];
  eul.pitch = euler[1];
  eul.yaw = euler[2];

  EA::Chronometer timer;
  timer.tic();

  namespace ph = std::placeholders;
  // Optimization for rotation alone
  GA_Type ga_obj;
  ga_obj.problem_mode = EA::GA_MODE::SOGA;
  ga_obj.multi_threading = false;
  ga_obj.verbose = false;
  ga_obj.population = 200;
  ga_obj.generation_max = 1000;
  ga_obj.calculate_SO_total_fitness = std::bind(&Optimiser::calculate_SO_total_fitness, this, ph::_1);
  ga_obj.init_genes = std::bind(&Optimiser::init_genes, this, ph::_1, ph::_2);
  ga_obj.eval_solution = std::bind(&Optimiser::eval_solution, this, ph::_1, ph::_2);
  ga_obj.mutate = std::bind(&Optimiser::mutate, this, ph::_1, ph::_2, ph::_3);
  ga_obj.crossover = std::bind(&Optimiser::crossover, this, ph::_1, ph::_2, ph::_3);
  ga_obj.SO_report_generation = std::bind(&Optimiser::SO_report_generation, this, ph::_1, ph::_2, ph::_3);
  ga_obj.best_stall_max = 100;
  ga_obj.average_stall_max = 100;
  ga_obj.tol_stall_average = 1e-8;
  ga_obj.tol_stall_best = 1e-8;
  ga_obj.elite_count = 10;
  ga_obj.crossover_fraction = 0.8;
  ga_obj.mutation_rate = 0.2;
  ga_obj.best_stall_max = 10;
  ga_obj.elite_count = 10;
  ga_obj.solve();

  // Optimized rotation
  tf::Matrix3x3 rot;
  rot.setRPY(eul_t.rot.roll, eul_t.rot.pitch, eul_t.rot.yaw);
  cv::Mat tmp_rot = (cv::Mat_<double>(3, 3) << rot.getRow(0)[0], rot.getRow(0)[1], rot.getRow(0)[2], rot.getRow(1)[0],
                     rot.getRow(1)[1], rot.getRow(1)[2], rot.getRow(2)[0], rot.getRow(2)[1], rot.getRow(2)[2]);
  // Analytical Translation
  cv::Mat cp_trans = tmp_rot * camera_centres_.t();
  cv::Mat trans_diff = lidar_centres_.t() - cp_trans;
  cv::Mat summed_diff;
  cv::reduce(trans_diff, summed_diff, 1, CV_REDUCE_SUM, CV_64F);
  summed_diff = summed_diff / trans_diff.cols;
  eul_t.x = summed_diff.at<double>(0);
  eul_t.y = summed_diff.at<double>(1);
  eul_t.z = summed_diff.at<double>(2);
  ROS_INFO_STREAM("Rotation and Translation after first optimization " << eul_t.rot.roll << " " << eul_t.rot.pitch
                                                                       << " " << eul_t.rot.yaw << " " << eul_t.x << " "
                                                                       << eul_t.y << " " << eul_t.z);

  // extrinsics stored the vector of extrinsic parameters in every iteration
  std::vector<std::vector<double>> extrinsics;
  for (int i = 0; i < 10; i++)
  {
    // Joint optimization for Rotation and Translation (Perform this 10 times and take the average of the extrinsics)
    GA_Type2 ga_obj2;
    ga_obj2.problem_mode = EA::GA_MODE::SOGA;
    ga_obj2.multi_threading = false;
    ga_obj2.verbose = false;
    ga_obj2.population = 200;
    ga_obj2.generation_max = 1000;
    ga_obj2.calculate_SO_total_fitness = std::bind(&Optimiser::calculate_SO_total_fitness2, this, ph::_1);
    ga_obj2.init_genes = std::bind(&Optimiser::init_genes2, this, ph::_1, ph::_2);
    ga_obj2.eval_solution = std::bind(&Optimiser::eval_solution2, this, ph::_1, ph::_2);
    ga_obj2.mutate = std::bind(&Optimiser::mutate2, this, ph::_1, ph::_2, ph::_3);
    ga_obj2.crossover = std::bind(&Optimiser::crossover2, this, ph::_1, ph::_2, ph::_3);
    ga_obj2.SO_report_generation = std::bind(&Optimiser::SO_report_generation2, this, ph::_1, ph::_2, ph::_3);
    ga_obj2.best_stall_max = 100;
    ga_obj2.average_stall_max = 100;
    ga_obj2.tol_stall_average = 1e-8;
    ga_obj2.tol_stall_best = 1e-8;
    ga_obj2.elite_count = 10;
    ga_obj2.crossover_fraction = 0.8;
    ga_obj2.mutation_rate = 0.2;
    ga_obj2.best_stall_max = 10;
    ga_obj2.elite_count = 10;
    ga_obj2.solve();
    std::vector<double> ex_it;
    ex_it.push_back(eul_it.rot.roll);
    ex_it.push_back(eul_it.rot.pitch);
    ex_it.push_back(eul_it.rot.yaw);
    ex_it.push_back(eul_it.x);
    ex_it.push_back(eul_it.y);
    ex_it.push_back(eul_it.z);
    extrinsics.push_back(ex_it);
  }
  // Perform the average operation
  double e_x = 0.0;
  double e_y = 0.0;
  double e_z = 0.0;
  double e_e1 = 0.0;
  double e_e2 = 0.0;
  double e_e3 = 0.0;
  for (int i = 0; i < 10; i++)
  {
    e_e1 += extrinsics[i][0];
    e_e2 += extrinsics[i][1];
    e_e3 += extrinsics[i][2];
    e_x += extrinsics[i][3];
    e_y += extrinsics[i][4];
    e_z += extrinsics[i][5];
  }

  RotationTranslation rot_trans;
  rot_trans.rot.roll = e_e1 / 10;
  rot_trans.rot.pitch = e_e2 / 10;
  rot_trans.rot.yaw = e_e3 / 10;
  rot_trans.x = e_x / 10;
  rot_trans.y = e_y / 10;
  rot_trans.z = e_z / 10;
  ROS_INFO_STREAM("Extrinsic Parameters "
                  << " " << rot_trans.rot.roll << " " << rot_trans.rot.pitch << " " << rot_trans.rot.yaw << " "
                  << rot_trans.x << " " << rot_trans.y << " " << rot_trans.z);
  ROS_INFO_STREAM("The problem was optimised in " << timer.toc() << " seconds");

  return true;
}

// Function converts rotation matrix to corresponding euler angles
std::vector<double> rotm2eul(cv::Mat mat)
{
  std::vector<double> euler(3);
  euler[0] = atan2(mat.at<double>(2, 1), mat.at<double>(2, 2));  // rotation about x axis: roll
  euler[1] = atan2(-mat.at<double>(2, 0),
                   sqrt(mat.at<double>(2, 1) * mat.at<double>(2, 1) + mat.at<double>(2, 2) * mat.at<double>(2, 2)));
  euler[2] = atan2(mat.at<double>(1, 0), mat.at<double>(0, 0));  // rotation about z axis: yaw
  return euler;
}

Optimiser::Optimiser(const initial_parameters_t& params) : i_params(params)
{
}
}  // namespace cam_lidar_calibration
