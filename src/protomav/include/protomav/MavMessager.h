#ifndef MAV_MESSENGER_HEADER
#define MAV_MESSENGER_HEADER

#include <mavros/Mavlink.h>
#include "mavlink/common/mavlink.h"
#include <ros/ros.h>
#include <ros/console.h>
#include "vehicle.h"

using namespace protomav;
class MavMessenger{
public:
  MavMessenger();
  void send(mavlink_message_t mmsg);
  virtual void send();
  float getFreq();

private:
  ros::NodeHandle nh;
  ros::Publisher mav_pub;
  int system_id, comp_id;

protected:
  float freq;
};
