#ifndef MAV_STATUS_HEADER
#define MAV_STATUS_HEADER

#include "MavMessenger.h"
const char* sensors_list_arr[] = {"xsens", "hokuyo"};
static std::vector<std::string> sensors_list(sensors_list_arr, sensors_list_arr+1);
static std::vector<uint32_t> sensors_mask (
      (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
      MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE | 
      MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_YAW_POSITION), //XSENS_MASK
  
      (MAV_SYS_STATUS_SENSOR_LASER_POSITION)				//HOKUYO_MASK
  
			     );	

namespace protomav{
class MavStatus : public protomav::MavMessenger{
public:
  MavStatus();
  virtual void send();
  int getLoad();
  uint32_t getSensorsPresent();
  uint32_t getSensorsEnabled();
  uint32_t getSensorsHealth();
  
private:
};
}

#endif