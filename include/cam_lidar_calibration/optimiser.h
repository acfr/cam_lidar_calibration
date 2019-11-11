#ifndef optimiser_h_
#define optimiser_h_

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
  Optimiser();
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
  double* convertToImagePoints(double x, double y, double z);

  double perpendicularCost(const Rotation& rot);
  double alignmentCost(const Rotation& rot);

  cv::Mat camera_normals_;
  cv::Mat camera_centres_;
  cv::Mat lidar_centres_;
  cv::Mat lidar_normals_;
  cv::Mat lidar_corners_;
  cv::Mat pixel_errors_;

  Rotation eul;
  RotationTranslation eul_t, eul_it;
  initial_parameters_t i_params;

  bool sensor_pair = 0;
  bool output = 0;
  bool output2 = 0;
  static cv::Mat new_K;
  ros::Subscriber calibdata_sub_;
};

std::vector<double> rotm2eul(cv::Mat);

}  // namespace cam_lidar_calibration

#endif
