#include "MavMessenger.h"

using namespace protomav;

MavMessenger::MavMessenger(){
  std::string mav_topic;
  ros::NodeHandle nh;
  nh.param<std::string>("mav_topic", mav_topic, "mavlink/from");
  nh.param<int>("system_id", system_id, 1);
  nh.param<int>("comp_id", comp_id, 240);
  mav_pub = nh.advertise<mavros_msgs::Mavlink>(mav_topic, 10);
}

float MavMessenger::getFreq(){
  return freq; 
}
 		
void MavMessenger::send(mavlink_message_t mmsg){
  mavros_msgs::Mavlink mav_msg;
  mav_msg.header.stamp=ros::Time::now(); 
  mav_msg.len = mmsg.len;
  mav_msg.seq = mmsg.seq;
  mav_msg.sysid = mmsg.sysid;
  mav_msg.compid = mmsg.compid;
  mav_msg.msgid = mmsg.msgid;
  mav_msg.payload64.reserve(((int) mmsg.len + 7) / 8);
  for (size_t i = 0; i < (mmsg.len + 7) / 8; i++){
  // ROS_INFO_STREAM(i << ":" <<(uint64_t)(mmsg.payload64[i]));
    mav_msg.payload64.push_back(mmsg.payload64[i]);
  }
  ROS_DEBUG("Sent Mavlink Message on ROS Topic, msgid: %d", mmsg.msgid);
  mav_pub.publish(mav_msg); 
}

void MavMessenger::run(){
 ros::Rate rate(getFreq());
 while(ros::ok()){
  send();
  rate.sleep();
  ros::spinOnce();
  } 
}
  


