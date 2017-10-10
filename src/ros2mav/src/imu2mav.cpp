#include "Imu2Mav.h"
Imu2Mav::Imu2Mav(){
 // ROS_INFO("CONSTRUCTOR");
  std::string imu_topic, mag_topic;
  ros::NodeHandle nh;
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("mag_topic", mag_topic);
  nh.param("imu_mav_freq", freq, 0.0);
  sync = (freq>0.0);
  
  imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &Imu2Mav::imuCallback, this);
  mag_sub = nh.subscribe<sensor_msgs::MagneticField>(mag_topic, 10, &Imu2Mav::magneticCallback, this);
  
}



void Imu2Mav::send(){
  mavlink_message_t mmsg; 

  ros::Time now = ros::Time::now();
  imu_scaled.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;
  
  mavlink_msg_scaled_imu_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &imu_scaled);
  MavMessenger::send(mmsg);
}


void Imu2Mav::imuCallback(const sensor_msgs::Imu::ConstPtr& imu){
  ROS_DEBUG("IMU CB");
  //data to be built upon Imu ROS topic  
  
  imu_scaled.xacc=imu->linear_acceleration.x*MS2_TO_MILLIG;
  imu_scaled.yacc=imu->linear_acceleration.y*MS2_TO_MILLIG;
  imu_scaled.zacc=imu->linear_acceleration.z*MS2_TO_MILLIG;
  
  
  imu_scaled.xgyro=imu->angular_velocity.x*1.0e3;
  imu_scaled.ygyro=imu->angular_velocity.y*1.0e3;
  imu_scaled.zgyro=imu->angular_velocity.z*1.0e3;
  
  
  if(!sync) {send();}
  
}

void Imu2Mav::magneticCallback(const sensor_msgs::MagneticField::ConstPtr& mag){
  ROS_DEBUG("MAGNETIC CB");
  imu_scaled.xmag=mag->magnetic_field.x*1.0e3;
  imu_scaled.ymag=mag->magnetic_field.y*1.0e3;
  imu_scaled.zmag=mag->magnetic_field.z*1.0e3;
  
  if(!sync) {send();}
}


int main(int argc,  char** argv) {
 ros::init(argc, argv, "imu2mav");

 Imu2Mav imu2mav;
 
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP"); 
}

