#include "alt2mav.h"
Alt2Mav::Alt2Mav(){
 // ROS_INFO("CONSTRUCTOR");
  std::string temp_topic, pressure_topic;
  ros::NodeHandle nh;
  nh.getParam("temp_topic", temp_topic);
  nh.getParam("pressure_topic", pressure_topic);
  nh.param("temp_mav_freq", freq, 0.0);
  sync = (freq>0.0);

  temp_sub = nh.subscribe<sensor_msgs::Temperature>(temp_topic, 10, &Temp2Mav::tempCallback, this);
  pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>(pressure_topic, 10, &Temp2Mav::pressureCallback, this);

}



void Alt2Mav::send(){
  mavlink_message_t mmsg;

  ros::Time now = ros::Time::now();
  mav_pressure.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;

  mavlink_msg_scaled_pressure_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &mav_pressure);
  MavMessenger::send(mmsg);
}


void Alt2Mav::altitudeCallback(const sensor_msgs::Temperature::ConstPtr& temp){
  mav_pressure.temperature = temp->temperature;
  if(!sync) {send();}
}




int main(int argc,  char** argv) {
 ros::init(argc, argv, "alt2mav");

 Alt2Mav alt2mav;

 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP");
}
