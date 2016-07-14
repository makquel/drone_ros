#ifndef MAV_MESSENGER_HEADER
#define MAV_MESSENGER_HEADER

#include <mavros_msgs/Mavlink.h>
#include "mavlink/common/mavlink.h"
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