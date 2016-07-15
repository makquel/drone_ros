#ifndef MAV_MESSENGER_HEADER
#define MAV_MESSENGER_HEADER

// #define MAVLINK_DIALECT common
#include <mavconn/interface.h>
// #include <mavlink/config.h>
#include <mavros_msgs/Mavlink.h>
#include <utility>
#include <mavros_msgs/mavlink_convert.h>
#include <mavlink/v1.0/common/mavlink.h>// #include <mavlink.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "vehicle.h"

namespace protomav{
  
  class MavMessenger{
  public:
    MavMessenger();
    void send(mavlink_message_t mmsg);
    virtual void send()=0;
    float getFreq();
    void run();

    
  protected:
    double freq;
    ros::Publisher mav_pub;
    int system_id, comp_id;
    bool sync;

  };
  
}
#endif