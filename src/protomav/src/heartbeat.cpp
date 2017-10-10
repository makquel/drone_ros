#include "MavHeartbeat.h"

using namespace protomav;

MavHeartbeat::MavHeartbeat(){
  ros::NodeHandle nh;
  nh.param<double>("hb_freq", freq, 1.0);
  pilot_mode_sub = nh.subscribe<ros_pololu_servo::DigitalState>("/pololu/digital_state", 100, &MavHeartbeat::muxCallback, this);

  mav_mode_ = 64; //	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 64
                //	MAV_MODE_FLAG_AUTO_ENABLED 4


}

void MavHeartbeat::send(){
  //std::string mav_mode = "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED";
  mavlink_message_t mmsg;
  // mavlink_heartbeat_t
  mavlink_msg_heartbeat_pack(SYSTEM_ID, COMP_ID, &mmsg,SYSTEM_TYPE, AUTOPILOT_TYPE,
			     mav_mode_, CUSTOM_MODE, SYSTEM_STATE);
  MavMessenger::send(mmsg);

}

void  MavHeartbeat::muxCallback(const ros_pololu_servo::DigitalState::ConstPtr& flight_mode)
{

  commutator = flight_mode->comutador;
  //ROS_INFO_THROTTLE(1, "commutator: %d", mav_mode_);
  if (commutator == true)
    mav_mode_ = 4;
    else
    mav_mode_ = 64;
    if(!sync) {send();}
}

int main(int argc,  char** argv) {
 ros::init(argc, argv, "mav_heartbeat");

 MavHeartbeat mhb;
 mhb.run();

 ros::spin();
 return 0;

}
