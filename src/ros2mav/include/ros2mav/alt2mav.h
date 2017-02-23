#ifndef ALT2MAV_HEADER
#define ALT2MAV_HEADER

//TODO:  Class
#include <protomav/MavMessenger.h>

#include <sf11_altimeter/sensor_data.h>


class alt2mav: public protomav::MavMessenger{
public:
  Alt2Mav();
  void send();

private:
  void altitudeCallback(const sf11_altimeter::sensor_data::ConstPtr& altitude);


  ros::Subscriber alt_sub;
  mavlink_scaled_pressure_t mav_pressure;

  const double MILLIG_TO_MS2 = 9.80665 / 1000.0;
  const double MS2_TO_MILLIG = 1/MILLIG_TO_MS2;

};
#endif
