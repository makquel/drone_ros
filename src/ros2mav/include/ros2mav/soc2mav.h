#ifndef SOC2MAV_HEADER
#define SOC2MAV_HEADER


#include <protomav/MavMessenger.h>
#include <sensor_msgs/BatteryState.h>
#include <yocto/voltage_info.h>


class SoC2Mav: public protomav::MavMessenger{
public:
  SoC2Mav();
  virtual void send();

private:
  void voltageCallback(const yocto::voltage_info::ConstPtr& voltage);


  ros::Subscriber voltage_sub;
  mavlink_battery_status_t mav_soc; // BATTERY_STATUS ( #147 )
  mavlink_battery_status_t mav_soc2; // BATTERY_STATUS ( #147 )
  //mavlink_sys_status_t status; // SYS_STATUS ( #74 )

};
#endif
