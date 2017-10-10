#ifndef ROS2ATT_HEADER
#define ROS2ATT_HEADER

#include <protomav/MavMessenger.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

class Ros2Att : public protomav::MavMessenger{
public:
  Ros2Att();
//  mavlink_scaled_imu_t* getMavlinkImuPtr();
 // mavlink_message_t* getMavlinkMsgPtr();
  void send();

private:
  void rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& rpy);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);


  //SUBSCRIBERS
  ros::Subscriber imu_sub;
  ros::Subscriber rpy_sub;

  //MAVLINK DATA TYPE
  mavlink_attitude_t att; //ATTITUDE #30

  //CONST FOR UNIT CONVERSION
  const double RAD_TO_DEG =180.0/3.1415;
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};

#endif
