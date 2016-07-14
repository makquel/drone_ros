#include "MavHeartbeat.h"
using namespace protomav;
MavHeartbeat::MavHeartbeat(){
  ros::NodeHandle nh("~");
  nh.param<double>("hb_freq", freq, 1.0);
}


void MavHeartbeat::send(){
  
  mavlink_message_t mmsg; 
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMP_ID, &mmsg,SYSTEM_TYPE, AUTOPILOT_TYPE,
			     SYSTEM_MODE, CUSTOM_MODE, SYSTEM_STATE);
  MavMessenger::send(mmsg);

}



int main(int argc,  char** argv) {
 ros::init(argc, argv, "mav_heartbeat");

 MavHeartbeat mhb;
  
 mhb.run();
 
 
}

