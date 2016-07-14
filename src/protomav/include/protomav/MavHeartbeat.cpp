#ifndef MAV_HEARTBEAT_HEADER
#define MAV_HEARTBEAT_HEADER

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <mavros/Mavlink.h>
#include "mavlink/common/mavlink.h"
#include <ros/ros.h>
#include <ros/console.h>
#include "vehicle.h"

class MavHeartbeat{
public:
  MavHeartbeat();
  void send_heartbeat();
  float getHbFreq();
  
private:
  ros::NodeHandle nh;
  ros::Publisher mav_pub;
  double hb_freq;
  // Define the system type, in this case an airplane
 
 
  
};

#endif
