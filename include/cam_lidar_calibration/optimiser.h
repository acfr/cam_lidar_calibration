#ifndef optimiser_h_
#define optimiser_h_

#include <opencv/cv.hpp>

#include <ros/ros.h>

#include "cam_lidar_calibration/load_params.h"
#include "cam_lidar_calibration/openga.h"

namespace cam_lidar_calibration
{
struct Rotation
{
  double roll;  // Rotation optimization variables
  double pitch;
  double yaw;
  std::string to_string() const
  {
    return std::string("{") + "roll:" + std::to_string(roll) + ", pitch:" + std::to_string(pitch) +
           ", yaw:" + std::to_string(yaw) + "}";
  }
  cv::Mat toMat() const
  {
    using cv::Mat_;
    using std::cos;
    using std::sin;

    cv::Mat R_x = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll));
    // Calculate rotation about y axis
    cv::Mat R_y = (Mat_<double>(3, 3) << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch));

    // Calculate rotation about z axis
    cv::Mat R_z = (Mat_<double>(3, 3) << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);

    return R_z * R_y * R_x;
  }
};

cv::Mat operator*(const Rotation& lhs, const cv::Point3d& rhs);

struct RotationTranslation
{
  Rotation rot;
  double x;
  double y;
  double z;
  std::string to_string() const
  {
    return std::string("{") + "roll:" + std::to_string(rot.roll) + ", pitch:" + std::to_string(rot.pitch) +
           ", yaw:" + std::to_string(rot.yaw) + ", x:" + std::to_string(x) + ", y:" + std::to_string(y) +
           ", z:" + std::to_string(z) + "}";
  }
};

cv::Mat operator*(const RotationTranslation& lhs, const cv::Point3d& rhs);

struct RotationCost  // equivalent to y in matlab
{
  double objective1;  // This is where the results of simulation is stored but not yet finalized.
};

struct RotationTranslationCost  // equivalent to y in matlab
{
  double objective2;  // This is where the results of simulation is stored but not yet finalized.
};

struct OptimisationSample
{
  cv::Point3d camera_centre{ 0, 0, 0 };
  cv::Point3d camera_normal{ 0, 0, 0 };
  std::vector<cv::Point3d> camera_corners;
  cv::Point3d lidar_centre{ 0, 0, 0 };
  cv::Point3d lidar_normal{ 0, 0, 0 };
  std::vector<cv::Point3d> lidar_corners;
};

typedef EA::Genetic<Rotation, RotationCost> GA_Type;
typedef EA::Genetic<RotationTranslation, RotationTranslationCost> GA_Type2;

class Optimiser
{
public:
  Optimiser(const initial_parameters_t& params);
  ~Optimiser() = default;

  bool optimise();
  std::vector<OptimisationSample> samples_;

  void SO_report_generation(int generation_number, const EA::GenerationType<Rotation, RotationCost>& last_generation,
                            const Rotation& best_genes);
  double calculate_SO_total_fitness(const GA_Type::thisChromosomeType& X);
  Rotation crossover(const Rotation& X1, const Rotation& X2, const std::function<double(void)>& rnd01);
  Rotation mutate(const Rotation& X_base, const std::function<double(void)>& rnd01, double shrink_scale);
  bool eval_solution(const Rotation& p, RotationCost& c);
  void init_genes(Rotation& p, const std::function<double(void)>& rnd01);

  void SO_report_generation2(int generation_number,
                             const EA::GenerationType<RotationTranslation, RotationTranslationCost>& last_generation,
                             const RotationTranslation& best_genes);
  double calculate_SO_total_fitness2(const GA_Type2::thisChromosomeType& X);
  RotationTranslation crossover2(const RotationTranslation& X1, const RotationTranslation& X2,
                                 const std::function<double(void)>& rnd01);
  RotationTranslation mutate2(const RotationTranslation& X_base, const std::function<double(void)>& rnd01,
                              double shrink_scale);
  bool eval_solution2(const RotationTranslation& p, RotationTranslationCost& c);
  void init_genes2(RotationTranslation& p, const std::function<double(void)>& rnd01);

private:
  double perpendicularCost(const Rotation& rot);
  double normalAlignmentCost(const Rotation& rot);
  double reprojectionCost(const RotationTranslation& rot_trans);
  double centreAlignmentCost(const RotationTranslation& rot_trans);

  Rotation eul;
  RotationTranslation eul_t, eul_it;
  initial_parameters_t i_params;
};

std::vector<double> rotm2eul(cv::Mat);

}  // namespace cam_lidar_calibration

#endif
