#ifndef GPS2MAV_HEADER
#define GPS2MAV_HEADER
#include <protomav/MavMessenger.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mtig_driver_msgs/GpsInfo.h>
#include <cmath>


class Gps2Mav: public protomav::MavMessenger{
public:
  Gps2Mav();
//  mavlink_scaled_imu_t* getMavlinkImuPtr();
 // mavlink_message_t* getMavlinkMsgPtr();
  void send();
  
private:
  void gpsInfoCallback(const mtig_driver_msgs::GpsInfo::ConstPtr& info);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps);
  void velCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel);
  ros::Subscriber gps_info_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber gps_sub;
  mavlink_global_position_int_t mgps;
  mavlink_gps_raw_int_t rgps;

  const double RAD_TO_DEG =180.0/3.1415;
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};

#endif