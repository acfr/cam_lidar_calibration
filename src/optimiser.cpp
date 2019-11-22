#include "cam_lidar_calibration/optimiser.h"

#include "cam_lidar_calibration/point_xyzir.h"

#include <tf/transform_datatypes.h>

namespace cam_lidar_calibration
{
cv::Mat operator*(const Rotation& lhs, const cv::Point3d& rhs)
{
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

void Optimiser::init_genes(RotationTranslation& p, const std::function<double(void)>& rnd,
                           const RotationTranslation& initial_rot_trans, double angle_increment,
                           double translation_increment)
{
  init_genes(p.rot, rnd, initial_rot_trans.rot, angle_increment);

  std::vector<double> trans_vals;
  trans_vals.push_back(translation_increment);
  trans_vals.push_back(-translation_increment);
  int RandIndex = rand() % 2;
  p.x = initial_rot_trans.x + trans_vals.at(RandIndex) * rnd();
  RandIndex = rand() % 2;
  p.y = initial_rot_trans.y + trans_vals.at(RandIndex) * rnd();
  RandIndex = rand() % 2;
  p.z = initial_rot_trans.z + trans_vals.at(RandIndex) * rnd();
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

bool Optimiser::eval_solution(const RotationTranslation& p, RotationTranslationCost& c)
{
  double perpendicular_cost = perpendicularCost(p.rot);
  double normal_align_cost = normalAlignmentCost(p.rot);

  double centre_align_cost = centreAlignmentCost(p);
  double repro_cost = reprojectionCost(p);
  c.objective2 = perpendicular_cost + normal_align_cost + centre_align_cost + repro_cost;

  return true;  // solution is accepted
}

RotationTranslation Optimiser::mutate(const RotationTranslation& X_base, const std::function<double(void)>& rnd,
                                      const RotationTranslation& initial_rotation_translation,
                                      const double angle_increment, const double translation_increment,
                                      const double shrink_scale)
{
  RotationTranslation X_new;
  X_new.rot = mutate(X_base.rot, rnd, initial_rotation_translation.rot, angle_increment, shrink_scale);
  bool in_range;
  do
  {
    in_range = true;
    X_new = X_base;
    X_new.x += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.x >= (initial_rotation_translation.x - 0.05) &&
                            X_new.x < (initial_rotation_translation.x + 0.05));
    X_new.y += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.y >= (initial_rotation_translation.y - 0.05) &&
                            X_new.y < (initial_rotation_translation.y + 0.05));
    X_new.z += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.z >= (initial_rotation_translation.z - 0.05) &&
                            X_new.z < (initial_rotation_translation.z + 0.05));

  } while (!in_range);
  return X_new;
}

RotationTranslation Optimiser::crossover(const RotationTranslation& X1, const RotationTranslation& X2,
                                         const std::function<double(void)>& rnd)
{
  RotationTranslation X_new;
  double r;
  r = rnd();
  X_new.rot = crossover(X1.rot, X2.rot, rnd);

  X_new.x = r * X1.x + (1.0 - r) * X2.x;
  r = rnd();
  X_new.y = r * X1.y + (1.0 - r) * X2.y;
  r = rnd();
  X_new.z = r * X1.z + (1.0 - r) * X2.z;
  return X_new;
}

double Optimiser::calculate_SO_total_fitness(const GA_Rot_Trans_t::thisChromosomeType& X)
{
  // finalize the cost
  double final_cost = 0.0;
  final_cost += X.middle_costs.objective2;
  return final_cost;
}

// A function to show/store the results of each generation.
void Optimiser::SO_report_generation(
    int generation_number, const EA::GenerationType<RotationTranslation, RotationTranslationCost>& last_generation,
    const RotationTranslation& best_genes)
{
  best_rotation_translation_.rot = best_genes.rot;
  best_rotation_translation_.x = best_genes.x;
  best_rotation_translation_.y = best_genes.y;
  best_rotation_translation_.z = best_genes.z;
}

void Optimiser::init_genes(Rotation& p, const std::function<double(void)>& rnd, const Rotation& initial_rotation,
                           double increment)
{
  std::vector<double> pi_vals;
  pi_vals.push_back(increment);
  pi_vals.push_back(-increment);
  int RandIndex = rand() % 2;
  p.roll = initial_rotation.roll + pi_vals.at(RandIndex) * rnd();
  RandIndex = rand() % 2;
  p.pitch = initial_rotation.pitch + pi_vals.at(RandIndex) * rnd();
  RandIndex = rand() % 2;
  p.yaw = initial_rotation.yaw + pi_vals.at(RandIndex) * rnd();
}

bool Optimiser::eval_solution(const Rotation& p, RotationCost& c)
{
  c.objective1 = perpendicularCost(p) + normalAlignmentCost(p);

  return true;  // solution is accepted
}

Rotation Optimiser::mutate(const Rotation& X_base, const std::function<double(void)>& rnd,
                           const Rotation& initial_rotation, const double angle_increment, double shrink_scale)
{
  Rotation X_new;
  bool in_range;
  do
  {
    in_range = true;
    X_new = X_base;
    X_new.roll += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.roll >= (initial_rotation.roll - angle_increment) &&
                            X_new.roll < (initial_rotation.roll + angle_increment));
    X_new.pitch += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.pitch >= (initial_rotation.pitch - angle_increment) &&
                            X_new.pitch < (initial_rotation.pitch + angle_increment));
    X_new.yaw += 0.2 * (rnd() - rnd()) * shrink_scale;
    in_range = in_range && (X_new.yaw >= (initial_rotation.yaw - angle_increment) &&
                            X_new.yaw < (initial_rotation.yaw + angle_increment));
  } while (!in_range);
  return X_new;
}

Rotation Optimiser::crossover(const Rotation& X1, const Rotation& X2, const std::function<double(void)>& rnd)
{
  Rotation X_new;
  double r = rnd();
  X_new.roll = r * X1.roll + (1.0 - r) * X2.roll;
  r = rnd();
  X_new.pitch = r * X1.pitch + (1.0 - r) * X2.pitch;
  r = rnd();
  X_new.yaw = r * X1.yaw + (1.0 - r) * X2.yaw;
  return X_new;
}

double Optimiser::calculate_SO_total_fitness(const GA_Rot_t::thisChromosomeType& X)
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
  best_rotation_.roll = best_genes.roll;
  best_rotation_.pitch = best_genes.pitch;
  best_rotation_.yaw = best_genes.yaw;
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

  const Rotation initial_rotation{ euler[0], euler[1], euler[2] };

  EA::Chronometer timer;
  timer.tic();

  double rotation_increment = M_PI / 8.;
  namespace ph = std::placeholders;
  // Optimization for rotation alone
  GA_Rot_t ga_obj;
  ga_obj.problem_mode = EA::GA_MODE::SOGA;
  ga_obj.multi_threading = false;
  ga_obj.verbose = false;
  ga_obj.population = 200;
  ga_obj.generation_max = 1000;
  ga_obj.calculate_SO_total_fitness = [&](const GA_Rot_t::thisChromosomeType& X) -> double {
    return this->calculate_SO_total_fitness(X);
  };
  ga_obj.init_genes = [&, initial_rotation, rotation_increment](Rotation& p,
                                                                const std::function<double(void)>& rnd) -> void {
    this->init_genes(p, rnd, initial_rotation, rotation_increment);
  };
  ga_obj.eval_solution = [&](const Rotation& r, RotationCost& c) -> bool { return this->eval_solution(r, c); };
  ga_obj.mutate = [&, initial_rotation, rotation_increment](
                      const Rotation& X_base, const std::function<double(void)>& rnd, double shrink_scale) -> Rotation {
    return this->mutate(X_base, rnd, initial_rotation, rotation_increment, shrink_scale);
  };
  ga_obj.crossover = [&](const Rotation& X1, const Rotation& X2, const std::function<double(void)>& rnd) {
    return this->crossover(X1, X2, rnd);
  };
  ga_obj.SO_report_generation = [&](int generation_number,
                                    const EA::GenerationType<Rotation, RotationCost>& last_generation,
                                    const Rotation& best_genes) -> void {
    this->SO_report_generation(generation_number, last_generation, best_genes);
  };
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
  // Reset starting point of rotation genes
  tf::Matrix3x3 rot;
  rot.setRPY(best_rotation_.roll, best_rotation_.pitch, best_rotation_.yaw);
  cv::Mat tmp_rot = (cv::Mat_<double>(3, 3) << rot.getRow(0)[0], rot.getRow(0)[1], rot.getRow(0)[2], rot.getRow(1)[0],
                     rot.getRow(1)[1], rot.getRow(1)[2], rot.getRow(2)[0], rot.getRow(2)[1], rot.getRow(2)[2]);
  // Analytical Translation
  cv::Mat cp_trans = tmp_rot * camera_centres_.t();
  cv::Mat trans_diff = lidar_centres_.t() - cp_trans;
  cv::Mat summed_diff;
  cv::reduce(trans_diff, summed_diff, 1, CV_REDUCE_SUM, CV_64F);
  summed_diff = summed_diff / trans_diff.cols;

  const RotationTranslation initial_rotation_translation{ best_rotation_, summed_diff.at<double>(0),
                                                          summed_diff.at<double>(1), summed_diff.at<double>(2) };

  ROS_INFO_STREAM("Rotation and Translation after first optimization "
                  << initial_rotation_translation.rot.roll << " " << initial_rotation_translation.rot.pitch << " "
                  << initial_rotation_translation.rot.yaw << " " << initial_rotation_translation.x / 1000.0 << " "
                  << initial_rotation_translation.y / 1000.0 << " " << initial_rotation_translation.z / 1000.0);

  rotation_increment = M_PI / 18.;
  constexpr double translation_increment = 0.05;
  // extrinsics stored the vector of extrinsic parameters in every iteration
  std::vector<std::vector<double>> extrinsics;
  for (int i = 0; i < 10; i++)
  {
    // Joint optimization for Rotation and Translation (Perform this 10 times and take the average of the extrinsics)
    GA_Rot_Trans_t ga_rot_trans;
    ga_rot_trans.problem_mode = EA::GA_MODE::SOGA;
    ga_rot_trans.multi_threading = false;
    ga_rot_trans.verbose = false;
    ga_rot_trans.population = 200;
    ga_rot_trans.generation_max = 1000;
    ga_rot_trans.calculate_SO_total_fitness = [&](const GA_Rot_Trans_t::thisChromosomeType& X) -> double {
      return this->calculate_SO_total_fitness(X);
    };
    ga_rot_trans.init_genes = [&, initial_rotation_translation, rotation_increment, translation_increment](
                                  RotationTranslation& p, const std::function<double(void)>& rnd) -> void {
      this->init_genes(p, rnd, initial_rotation_translation, rotation_increment, translation_increment);
    };
    ga_rot_trans.eval_solution = [&](const RotationTranslation& rt, RotationTranslationCost& c) -> bool {
      return this->eval_solution(rt, c);
    };
    ga_rot_trans.mutate = [&, initial_rotation_translation, rotation_increment, translation_increment](
                              const RotationTranslation& X_base, const std::function<double(void)>& rnd,
                              double shrink_scale) -> RotationTranslation {
      return this->mutate(X_base, rnd, initial_rotation_translation, rotation_increment, translation_increment,
                          shrink_scale);
    };
    ga_rot_trans.crossover = [&](const RotationTranslation& X1, const RotationTranslation& X2,
                                 const std::function<double(void)>& rnd) { return this->crossover(X1, X2, rnd); };
    ga_rot_trans.SO_report_generation =
        [&](int generation_number,
            const EA::GenerationType<RotationTranslation, RotationTranslationCost>& last_generation,
            const RotationTranslation& best_genes) -> void {
      this->SO_report_generation(generation_number, last_generation, best_genes);
    };
    ga_rot_trans.best_stall_max = 100;
    ga_rot_trans.average_stall_max = 100;
    ga_rot_trans.tol_stall_average = 1e-8;
    ga_rot_trans.tol_stall_best = 1e-8;
    ga_rot_trans.elite_count = 10;
    ga_rot_trans.crossover_fraction = 0.8;
    ga_rot_trans.mutation_rate = 0.2;
    ga_rot_trans.best_stall_max = 10;
    ga_rot_trans.elite_count = 10;
    ga_rot_trans.solve();
    std::vector<double> ex_it;
    ex_it.push_back(best_rotation_translation_.rot.roll);
    ex_it.push_back(best_rotation_translation_.rot.pitch);
    ex_it.push_back(best_rotation_translation_.rot.yaw);
    ex_it.push_back(best_rotation_translation_.x);
    ex_it.push_back(best_rotation_translation_.y);
    ex_it.push_back(best_rotation_translation_.z);
    extrinsics.push_back(ex_it);
  }
  // Perform the average operation
  double e_x = 0.0;
  double e_y = 0.0;
  double e_z = 0.0;
  double e_pitch = 0.0;
  double e_roll = 0.0;
  double e_yaw = 0.0;
  for (int i = 0; i < 10; i++)
  {
    e_pitch += extrinsics[i][0];
    e_roll += extrinsics[i][1];
    e_yaw += extrinsics[i][2];
    e_x += extrinsics[i][3];
    e_y += extrinsics[i][4];
    e_z += extrinsics[i][5];
  }

  RotationTranslation rot_trans;
  rot_trans.rot.roll = e_pitch / 10;
  rot_trans.rot.pitch = e_roll / 10;
  rot_trans.rot.yaw = e_yaw / 10;
  rot_trans.x = e_x / 10;
  rot_trans.y = e_y / 10;
  rot_trans.z = e_z / 10;
  ROS_INFO_STREAM("Extrinsic Parameters "
                  << " " << rot_trans.rot.roll << " " << rot_trans.rot.pitch << " " << rot_trans.rot.yaw << " "
                  << rot_trans.x / 1000.0 << " " << rot_trans.y / 1000.0 << " " << rot_trans.z / 1000.0);
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
