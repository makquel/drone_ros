#include "Gps2Mav.h"

Gps2Mav::Gps2Mav(){
  
  std::string gps_info_topic, vel_topic, gps_topic;
  ros::NodeHandle nh;
  nh.getParam("gps_info_topic", gps_info_topic);
  nh.getParam("vel_topic", vel_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.param("gps_mav_freq", freq, 0.0);
  sync=(freq>0.0);
  
  vel_sub = nh.subscribe<geometry_msgs::TwistWithCovariance>(vel_topic, 10, &Gps2Mav::velCallback, this);
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 10, &Gps2Mav::gpsCallback, this);
  
}



void Gps2Mav::send(){
  mavlink_message_t mmsg; 

  
  //TODO ALTITUDE RELATIVA
  mgps.relative_alt = __UINT32_MAX__;
  mgps.hdg = __UINT16_MAX__;
  ros::Time now = ros::Time::now();
  
  mgps.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;
  mavlink_msg_global_position_int_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &mgps);
 
  MavMessenger::send(mmsg);
}

void Gps2Mav::gpsInfoCallback(const mtig_driver_msgs::GpsInfo::ConstPtr& info){
  //TODO
}

void Gps2Mav::velCallback(const geometry_msgs::TwistWithCovariance::ConstPtr& vel){

  mgps.vx=vel->twist.linear.x;
  mgps.vy=vel->twist.linear.y;
  mgps.vz=vel->twist.linear.z;
  
  if(!sync){send();}
}

void Gps2Mav::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps){
  //mgps.fix_type=gps->status.status+3;
  mgps.lat= gps->latitude*1.0e7;
  mgps.lon= gps->longitude*1.0e7;
  mgps.alt = gps->altitude*1.0e3;
  
  if(!sync){send();}
}

int main(int argc,  char** argv) {
 ros::init(argc, argv, "gps2mav");

 Gps2Mav gps2mav;
 
 
 
 
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP"); 
}

