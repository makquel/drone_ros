#include "Ros2Att.h"

Ros2Att::Ros2Att(){
  ROS_INFO("ATT_CONSTRUCTOR");
  std::string  rpy_topic, imu_topic;
  ros::NodeHandle nh;

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("rpy_topic", rpy_topic);
  nh.param("att_mav_freq", freq, 0.0);
  sync = (freq>0.0);

  rpy_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(rpy_topic, 10, &Ros2Att::rpyCallback, this);
  imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 10, &Ros2Att::imuCallback, this);


}

void Ros2Att::send(){

  mavlink_message_t mmsg;

  ros::Time now=ros::Time::now();

  att.time_boot_ms=(uint64_t)now.sec*1000+(uint64_t)now.nsec/1.0e6;

  mavlink_msg_attitude_encode( (uint8_t) 1,  (uint8_t) 240, &mmsg,  &att);

  MavMessenger::send(mmsg);
}


void Ros2Att::imuCallback(const sensor_msgs::Imu::ConstPtr& imu){


  att.rollspeed=imu->angular_velocity.x;
  att.pitchspeed=imu->angular_velocity.y;
  att.yawspeed=imu->angular_velocity.z;

  if(!sync) send();
}

void Ros2Att::rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr& rpy){

    //XSENS (ENU) TO MAV(NED)
    att.roll=rpy->vector.x/RAD_TO_DEG;			// 1/RAD_TO_DEG = DEG_TO_RAD
    att.pitch=-rpy->vector.y/RAD_TO_DEG;
    att.yaw=(90.0-rpy->vector.z)/RAD_TO_DEG;		//Heading adjustment from XSENS
    if(!sync) send();
}




int main(int argc,  char** argv) {
 ros::init(argc, argv, "ros2att");

 Ros2Att Ros2Att;




 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP");
}
