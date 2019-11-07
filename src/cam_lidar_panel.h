#ifndef cam_lidar_panel_h_
#define cam_lidar_panel_h_

#ifndef Q_MOC_RUN
#include <ros/ros.h>

#include <rviz/panel.h>
#endif

#include <QPushButton>

namespace cam_lidar_calibration
{
class CamLidarPanel : public rviz::Panel
{
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  CamLidarPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

public Q_SLOTS:

protected Q_SLOTS:
  void captureSample();
  void discardSample();
  void optimise();

protected:
  // The ROS node handle.
  ros::NodeHandle nh_;
  ros::ServiceClient optimise_client_;

  QPushButton* capture_button_;
  QPushButton* discard_button_;
  QPushButton* optimise_button_;
};

}  // end namespace cam_lidar_calibration

#endif  // cam_lidar_panel_h_
