#ifndef SOC2MAV_HEADER
#define SOC2MAV_HEADER


#include <protomav/MavMessenger.h>
#include <sensor_msgs/FluidPressure.h>



class SoC2Mav: public protomav::MavMessenger{
public:
  SoC2Mav();
  void send();
  
private:
  void voltageCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure);
  
 
  ros::Subscriber voltage_sub;
  mavlink_battery_status_t mav_soc;
  
};
#endif