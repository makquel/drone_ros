#ifndef ROS2VFR_HEADER
#define ROS2VFR_HEADER

#include <protomav/MavMessenger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <tf/tf.h>
#include <cmath>


class Ros2Vfr: public protomav::MavMessenger{
public:
  Ros2Vfr();
//  mavlink_scaled_imu_t* getMavlinkImuPtr();
 // mavlink_message_t* getMavlinkMsgPtr();
  void send();
  
private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps);
  void velCallback(const geometry_msgs::TwistWithCovariance::ConstPtr& vel);
  
  ros::Subscriber imu_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber gps_sub;
  
  mavlink_vfr_hud_t vfr;

  const double RAD_TO_DEG =180.0/3.1415;
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};

#endif