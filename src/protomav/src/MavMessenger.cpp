#include "MavMessenger.h"

using namespace protomav;

MavMessenger::MavMessenger(){
  std::string mav_topic;
  ros::NodeHandle nh;
  nh.param<std::string>("mav_topic", mav_topic, "mavlink/from");
  nh.param<int>("system_id", system_id, 1); //QGS should have a diferent ID number
  nh.param<int>("comp_id", comp_id, 240);
  mav_pub = nh.advertise<mavros_msgs::Mavlink>(mav_topic, 10);
}

float MavMessenger::getFreq(){
  return freq;
}

void MavMessenger::send(mavlink_message_t mmsg){
  mavros_msgs::Mavlink rmsg;
  mavros_msgs::mavlink::convert(mmsg, rmsg);
  ROS_DEBUG("Sent Mavlink Message on ROS Topic, msgid: %d", mmsg.msgid);
  mav_pub.publish(rmsg);
}

void MavMessenger::run(){
 ros::Rate rate(getFreq());
 while(ros::ok()){
  send();
  rate.sleep();
  ros::spinOnce();
  }
}
