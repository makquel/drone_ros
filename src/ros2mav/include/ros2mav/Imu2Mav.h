#ifndef IMU2MAV_HEADER
#define IMU2MAV_HEADER


#include <protomav/MavMessenger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>


class Imu2Mav: public protomav::MavMessenger{
public:
  Imu2Mav();
  void send();
  
private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
  void magneticCallback( const sensor_msgs::MagneticField::ConstPtr& mag);
 
  ros::Subscriber imu_sub;
  ros::Subscriber mag_sub;
  mavlink_scaled_imu_t imu_scaled;
  
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};
#endif