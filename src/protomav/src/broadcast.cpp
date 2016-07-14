#include "MavBroadcast.h"
using namespace protomav;
MavBroadcast::MavBroadcast(){}

void MavBroadcast::reached(int seq){
  mavlink_mission_item_reached_t reached;
  reached.seq= seq;
  mavlink_msg_mission_item_reached_encode(SYSTEM_ID, COMP_ID, &out_msg, &reached);
  send();
}

void MavBroadcast::current(int seq){
  mavlink_mission_current_t current;
  current.seq=seq;
  mavlink_msg_mission_current_encode(SYSTEM_ID, COMP_ID, &out_msg, &current);
  send();
}

void MavBroadcast::send(){
  MavMessenger::send(out_msg);
}