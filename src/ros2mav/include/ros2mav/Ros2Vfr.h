#ifndef ROS2VFR_HEADER
#define ROS2VFR_HEADER

#include <protomav/MavMessenger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/FluidPressure.h>
#include <sf11_altimeter/sensor_data.h>
#include <droni_airspeed_driver/sensor_data.h>
//#include <yocto/PWM_info.h>
//#include <sensor_msgs/Temperature.h>
#include <tf/tf.h>
#include <cmath>
#include <tgmath.h>



class Ros2Vfr: public protomav::MavMessenger{
public:
  Ros2Vfr();
  //  mavlink_scaled_imu_t* getMavlinkImuPtr();
  //  mavlink_message_t* getMavlinkMsgPtr();
  void send();

private:
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  void velCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel);

  //Inserir um parâmetro para visualizar a altura relativa (alt. laser) ou altura absoluta (alt. barométrico)
  //void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps);
  //void gpsCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure);
  void altCallback(const sf11_altimeter::sensor_data::ConstPtr& alt);
  //TODO
  void airspeedCallback(const droni_airspeed_driver::sensor_data::ConstPtr& gspeed);
  //void throttleCallback(const yocto_master::PWM_info::ConstPtr& xthrottle);

  ros::Subscriber imu_sub;
  ros::Subscriber vel_sub;
  ros::Subscriber gps_sub;
  ros::Subscriber press_sub;
  ros::Subscriber alt_sub;
  ros::Subscriber airspeed_sub;

  mavlink_vfr_hud_t vfr; //SYS_STATUS ( #74 )

  const double RAD_TO_DEG =180.0/3.1415;
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};

#endif
