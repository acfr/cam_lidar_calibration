#ifndef optimiser_h_
#define optimiser_h_

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>

namespace cam_lidar_calibration
{
struct Rotation
{
  double e1;  // Rotation optimization variables
  double e2;
  double e3;
  std::string to_string() const
  {
    return std::string("{") + "e1:" + std::to_string(e1) + ", e2:" + std::to_string(e2) + ", e3:" + std::to_string(e3) +
           "}";
  }
};

struct RotationTranslation
{
  double e1;  // Joint (Rotation and translation) optimization variables
  double e2;
  double e3;
  double x;
  double y;
  double z;
  std::string to_string() const
  {
    return std::string("{") + "e1:" + std::to_string(e1) + ", e2:" + std::to_string(e2) + ", e3:" + std::to_string(e3) +
           ", x:" + std::to_string(x) + ", y:" + std::to_string(y) + ", z:" + std::to_string(z) + "}";
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

typedef EA::Genetic<Rotation, RotationCost> GA_Type;
typedef EA::Genetic<RotationTranslation, RotationTranslationCost> GA_Type2;

class Optimiser
{
public:
  Optimiser();
  ~Optimiser() = default;

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
  void getSamples(const cam_lidar_calibration::calibration_data::ConstPtr& data);
  double rotationFitnessFunc(double e1, double e2, double e3);

  void imageProjection(RotationTranslation rot_trans);
  bool optimiseCB(cam_lidar_calibration::Sample::Request& req, cam_lidar_calibration::Sample::Response& res);
  void sensorInfoCB(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::PointCloud2::ConstPtr& pc);

  Rotation eul;
  RotationTranslation eul_t, eul_it;
  int sample = 0;
  bool sensor_pair = 0;
  bool output = 0;
  bool output2 = 0;
  static cv::Mat new_K;
  cv::Mat raw_image, undist_image;
  pcl::PointCloud<pcl::PointXYZIR>::Ptr cloud;
  image_transport::Publisher pub_img_dist;
  cv_bridge::CvImagePtr cv_ptr;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2>
      ImageLidarSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ImageLidarSyncPolicy>> sync;
  ros::ServiceServer optimise_service_;
  ros::Subscriber calibdata_sub_;
};

std::vector<double> rotm2eul(cv::Mat);
pcl::PointCloud<pcl::PointXYZIR> organizedPointcloud(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud);

}  // namespace cam_lidar_calibration

#endif
