#include <gtest/gtest.h>

#include "cam_lidar_calibration/load_params.h"
#include "cam_lidar_calibration/optimiser.h"

using namespace cam_lidar_calibration;
using namespace cv;

auto createSamples = [](int count) {
  std::vector<OptimisationSample> samples;

  OptimisationSample s;
  s.camera_normal = Point3d(-0.553537, -0.0230432, -0.832505);
  s.camera_centre = Point3d(862.356, 83.6577, 2358.1);
  s.camera_corners.push_back(Point3d(884.857, -458.79, 2358.16));
  s.camera_corners.push_back(Point3d(437.634, 277.619, 2635.13));
  s.camera_corners.push_back(Point3d(1287.08, -110.304, 2081.07));
  s.camera_corners.push_back(Point3d(839.854, 626.106, 2358.05));
  s.lidar_normal = Point3d(-0.34838, 0.936304, -0.0443391);
  s.lidar_centre = Point3d(1502.56, -2137.06, -57.7846);
  s.lidar_corners.push_back(Point3d(998.856, -2318.25, 73.655));
  s.lidar_corners.push_back(Point3d(1398.51, -2149.77, 491.443));
  s.lidar_corners.push_back(Point3d(1998.63, -1957.96, -173.489));
  s.lidar_corners.push_back(Point3d(1614.26, -2122.26, -622.747));
  samples.push_back(s);

  if (count < 2)
    return samples;
  s.camera_normal = Point3d(-0.351556, -0.029757, -0.935694);
  s.camera_centre = Point3d(80.5651, 93.5535, 3419.82);
  s.camera_corners.push_back(Point3d(107.726, -448.636, 3426.86));
  s.camera_corners.push_back(Point3d(-397.497, 283.512, 3593.4));
  s.camera_corners.push_back(Point3d(558.628, -96.4048, 3246.25));
  s.camera_corners.push_back(Point3d(53.4043, 635.743, 3412.79));
  s.lidar_normal = Point3d(-0.546148, 0.835692, -0.0577988);
  s.lidar_centre = Point3d(2811.41, -2125.12, 105.91);
  s.lidar_corners.push_back(Point3d(2372.3, -2401.89, 253.491));
  s.lidar_corners.push_back(Point3d(2694.58, -2165.51, 625.87));
  s.lidar_corners.push_back(Point3d(3286.11, -1822.92, -10.0884));
  s.lidar_corners.push_back(Point3d(2892.65, -2110.17, -445.608));
  samples.push_back(s);

  if (count < 3)
    return samples;
  s.camera_normal = Point3d(0.77516, -0.0494489, -0.629827);
  s.camera_centre = Point3d(-2287.59, 137.276, 3153.72);
  s.camera_corners.push_back(Point3d(-2286.98, -403.913, 3196.96));
  s.camera_corners.push_back(Point3d(-2603.76, 316.749, 2750.51));
  s.camera_corners.push_back(Point3d(-1971.42, -42.1969, 3556.94));
  s.camera_corners.push_back(Point3d(-2288.2, 678.466, 3110.49));
  s.lidar_normal = Point3d(-0.968748, -0.24062, -0.0602387);
  s.lidar_centre = Point3d(3966.33, -27.9274, 254.86);
  s.lidar_corners.push_back(Point3d(4079.01, -531.781, 455.466));
  s.lidar_corners.push_back(Point3d(3938.84, -55.7691, 808.135));
  s.lidar_corners.push_back(Point3d(3854.27, 473.949, 52.3765));
  s.lidar_corners.push_back(Point3d(3993.21, 1.8917, -296.538));
  samples.push_back(s);

  if (count < 4)
    return samples;
  s.camera_normal = Point3d(0.257226, -0.0508405, -0.965013);
  s.camera_centre = Point3d(-1258.54, 117.407, 3462.74);
  s.camera_corners.push_back(Point3d(-1237.73, -424.038, 3496.82));
  s.camera_corners.push_back(Point3d(-1749.51, 301.638, 3322.17));
  s.camera_corners.push_back(Point3d(-767.567, -66.8243, 3603.32));
  s.camera_corners.push_back(Point3d(-1279.35, 658.852, 3428.67));
  s.lidar_normal = Point3d(-0.926893, 0.372328, -0.0473505);
  s.lidar_centre = Point3d(3632.97, -1055.55, 216.876);
  s.lidar_corners.push_back(Point3d(3430.47, -1539.84, 372.701));
  s.lidar_corners.push_back(Point3d(3585.3, -1103.64, 771.808));
  s.lidar_corners.push_back(Point3d(3833.92, -575.185, 60.4742));
  s.lidar_corners.push_back(Point3d(3682.18, -1003.55, -337.478));
  samples.push_back(s);
  /*
    s.camera_normal = Point3d();
    s.camera_centre = Point3d();
    s.camera_corners.push_back(Point3d());
    s.camera_corners.push_back(Point3d());
    s.camera_corners.push_back(Point3d());
    s.camera_corners.push_back(Point3d());
    s.lidar_normal = Point3d();
    s.lidar_centre = Point3d();
    s.lidar_corners.push_back(Point3d());
    s.lidar_corners.push_back(Point3d());
    s.lidar_corners.push_back(Point3d());
    s.lidar_corners.push_back(Point3d());
    samples.push_back(s);
    */

  return samples;
};

auto createParams = []() {
  initial_parameters_t params;
  params.fisheye_model = true;
  params.cameramat = (Mat_<double>(3, 3) << 1178.654264230649, 0.0, 960.769818463969, 0.0, 1182.0760126922614,
                      603.9601849872713, 0.0, 0.0, 1.0);
  params.distcoeff =
      (Mat_<double>(1, 4) << -0.09405418270319424, 0.08610743090764036, -0.17081566190501285, 0.09818990340541457);
  return params;
};

TEST(OptimiserTest, fullRunTest)
{
  Optimiser o(createParams());
  o.samples_ = createSamples(99);
  o.optimise();
  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
