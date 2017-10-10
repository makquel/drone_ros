#include "Gps2Mav.h"

Gps2Mav::Gps2Mav(){

  std::string gps_info_topic, vel_topic, gps_topic;
  ros::NodeHandle nh;
  nh.getParam("gps_info_topic", gps_info_topic);
  nh.getParam("vel_topic", vel_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.param("gps_mav_freq", freq, 0.0);
  sync=(freq>0.0);

  vel_sub = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(vel_topic, 10, &Gps2Mav::velCallback, this);
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 10, &Gps2Mav::gpsCallback, this);

}



void Gps2Mav::send(){
  // GLOBAL_POSITION_INT message
  mavlink_message_t mmsg;


  //TODO ALTITUDE RELATIVA
  mgps.relative_alt = __UINT32_MAX__;
  mgps.hdg = __UINT16_MAX__;
  ros::Time now = ros::Time::now();

  mgps.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;
  mavlink_msg_global_position_int_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &mgps);

  MavMessenger::send(mmsg);
  // GPS_RAW_INT message
  mavlink_message_t raw_mmsg;

  rgps.time_usec = mgps.time_boot_ms*1000;
  mavlink_msg_gps_raw_int_encode((uint8_t) 1,  (uint8_t) 240, &raw_mmsg, &rgps);
  MavMessenger::send(raw_mmsg);
}

void Gps2Mav::gpsInfoCallback(const mtig_driver_msgs::GpsInfo::ConstPtr& info){
  //TODO
  rgps.satellites_visible = info->satellite_number;
  rgps.fix_type = info->gps_fix;
}

void Gps2Mav::velCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel){

  mgps.vx=vel->twist.twist.linear.x;
  mgps.vy=vel->twist.twist.linear.y;
  mgps.vz=vel->twist.twist.linear.z;

  if(!sync){send();}
}

void Gps2Mav::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps){
  //rgps.fix_type=gps->status.status+3;
  mgps.lat= gps->latitude*1.0e7;
  mgps.lon= gps->longitude*1.0e7;
  //mgps.lat= 22.852071*-1;
  //mgps.lon= 47.127206*-1;
  mgps.alt = gps->altitude*1.0e3;

  rgps.lat = mgps.lat;
  rgps.lon = mgps.lon;
  rgps.alt = rgps.alt;

  //HDOP
  rgps.eph = __UINT16_MAX__;

  //VDOP
  rgps.epv = __UINT16_MAX__;

  //COG - Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
  rgps.cog = __UINT16_MAX__;

  if(!sync){send();}
}

int main(int argc,  char** argv) {
 ros::init(argc, argv, "gps2mav");

 Gps2Mav gps2mav;




 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP");
}
