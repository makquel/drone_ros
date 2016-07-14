#ifndef TEMP2MAV_HEADER
#define TEMP2MAV_HEADER


#include <protomav/MavMessenger.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>


class Temp2Mav: public protomav::MavMessenger{
public:
  Temp2Mav();
  void send();
  
private:
  void pressureCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure);
  void tempCallback( const sensor_msgs::Temperature::ConstPtr& temp);
 
  ros::Subscriber temp_sub;
  ros::Subscriber pressure_sub;
  mavlink_scaled_pressure_t mav_pressure;
  
  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};
#endif