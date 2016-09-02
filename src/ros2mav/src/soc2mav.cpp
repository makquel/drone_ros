#include "soc2mav.h"

SoC2Mav::SoC2Mav(){
 // ROS_INFO("CONSTRUCTOR");
  std::string voltage_topic;
  ros::NodeHandle nh;
  nh.getParam("pressure_topic", voltage_topic);
  
  nh.param("temp_mav_freq", freq, 0.0);
  sync = (freq>0.0);
  
  voltage_sub = nh.subscribe<sensor_msgs::FluidPressure>(voltage_topic, 10, &SoC2Mav::voltageCallback, this);  
}



void SoC2Mav::send(){
  mavlink_message_t mmsg; 

  ros::Time now = ros::Time::now();
  //mav_soc.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;
  
  //mavlink_msg_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_status_t* battery_status)
  mavlink_msg_battery_status_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &mav_soc);
  MavMessenger::send(mmsg);
}


void SoC2Mav::voltageCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure){
 
  mav_soc.battery_remaining = 90; 
  if(!sync) {send();}
}


int main(int argc,  char** argv) {
 ros::init(argc, argv, "soc2mav");

 SoC2Mav soc2mav;
 
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP"); 
}
