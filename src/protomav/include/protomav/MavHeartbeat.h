#ifndef MAV_HEARTBEAT_HEADER
#define MAV_HEARTBEAT_HEADER

#include "MavMessenger.h"
#include <ros_pololu_servo/DigitalState.h>

namespace protomav{
class MavHeartbeat : public protomav::MavMessenger{
public:
  MavHeartbeat();
  virtual void send();
  bool commutator; //Comutador
  uint8_t mav_mode_;

private:
  ros::NodeHandle nh;
  ros::Publisher mav_pub;
  double hb_freq;
  // Define the system type, in this case an airplane

  void muxCallback(const ros_pololu_servo::DigitalState::ConstPtr& flight_mode);
  ros::Subscriber pilot_mode_sub;

};
}
#endif
