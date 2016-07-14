#ifndef MAV_HEARTBEAT_HEADER
#define MAV_HEARTBEAT_HEADER

#include "MavMessenger.h"

namespace protomav{
class MavHeartbeat : public protomav::MavMessenger{
public:
  MavHeartbeat();
  virtual void send();
  
  
private:
  ros::NodeHandle nh;
  ros::Publisher mav_pub;
  double hb_freq;
  // Define the system type, in this case an airplane
 
 
  
};
}
#endif
