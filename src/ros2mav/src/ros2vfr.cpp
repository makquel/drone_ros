#include "Ros2Vfr.h"

Ros2Vfr::Ros2Vfr(){
  ROS_INFO("CONSTRUCTOR");
  std::string imu_topic, vel_topic,gps_topic;
  ros::NodeHandle nh;
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("vel_topic", vel_topic);
  nh.getParam("gps_topic", gps_topic);
  nh.param("vfr_mav_freq", freq, 0.0);
  sync = (freq>0.0);
  
  vel_sub = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>(vel_topic, 10, &Ros2Vfr::velCallback, this);
  imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &Ros2Vfr::imuCallback, this);
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 10, &Ros2Vfr::gpsCallback, this);
  
}



void Ros2Vfr::send(){
  mavlink_message_t mmsg; 
  mavlink_msg_vfr_hud_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &vfr);
  MavMessenger::send(mmsg);
}

void Ros2Vfr::imuCallback(const sensor_msgs::Imu::ConstPtr& imu){
    
  tf::Quaternion local_w(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
   //RPY in ENU frame
  double groll, gpitch, gyaw;
  tf::Matrix3x3(local_w).getRPY(groll,gpitch, gyaw);
  
  //data to be built upon Imu ROS topic  
  double q1=imu->orientation.x;
  double q2=imu->orientation.y;
  double q3=imu->orientation.z;
  double q0=imu->orientation.w;
  //AJUSTE DE NOVENTA GRAUS NECESSARIO CONFORME MANUAL DO XSENS. TODO Ajuste deve ser feito antes talvez.
  vfr.heading = ((int) (90-(atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))*RAD_TO_DEG)) )% 360;
  if(vfr.heading>180) vfr.heading -=360;

  if(!sync) send();
}

void Ros2Vfr::velCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& vel){
  vfr.groundspeed=vfr.airspeed=sqrt(vel->twist.twist.linear.z*vel->twist.twist.linear.z
			       +vel->twist.twist.linear.y*vel->twist.twist.linear.y
			       +vel->twist.twist.linear.x*vel->twist.twist.linear.x);
  
  vfr.climb=vel->twist.twist.linear.z;
  if(!sync) send();
}

void Ros2Vfr::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps){
  vfr.alt = gps->altitude; 
  if(!sync) send();
}

int main(int argc,  char** argv) {
 ros::init(argc, argv, "ros2vfr");

 Ros2Vfr ros2vfr;
 
 
 
 
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP"); 
}

