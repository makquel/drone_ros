#include "soc2mav.h"

SoC2Mav::SoC2Mav(){
  ROS_INFO("SOC_CONSTRUCTOR");
  std::string voltage_topic;
  ros::NodeHandle nh;
  nh.getParam("voltage_topic", voltage_topic);

  nh.param("soc_mav_freq", freq, 0.0);
  sync = (freq>0.0);
  voltage_sub = nh.subscribe<yocto::voltage_info>(voltage_topic, 100, &SoC2Mav::voltageCallback, this);
}



void SoC2Mav::send(){
  mavlink_message_t mmsg;

  ros::Time now = ros::Time::now();
  //mav_soc.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;

  //mavlink_msg_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_status_t* battery_status)
  mavlink_msg_battery_status_encode( system_id,  comp_id, &mmsg,  &mav_soc);
  MavMessenger::send(mmsg);
}


void SoC2Mav::voltageCallback(const yocto::voltage_info::ConstPtr& voltage){
  mav_soc.id = 1; //On board computer
  mav_soc.type = 4; //MAV_BATTERY_TYPE_NIMH
  mav_soc.battery_function = 4; //MAV_BATTERY_TYPE_PAYLOAD
  //float on_board_battery_voltage = 16;
  //mav_soc.battery_remaining = on_board_battery_voltage*100/18.5;
  mav_soc.battery_remaining = 80;
  if(!sync) {send();}
}


int main(int argc,  char** argv) {
 ros::init(argc, argv, "soc2mav");

 SoC2Mav soc2mav;
 soc2mav.run();
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP");
}
