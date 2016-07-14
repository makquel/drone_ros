#include "MavControl.h"

using namespace protomav;
MavControl::MavControl(){
 wp_id=0;
 ros::NodeHandle nh;
 std::string mav_topic, wp_frame, planner_frame;
 nh.getParam("waypoint_frame", wp_frame);
 nh.getParam("planner_frame", planner_frame);
 nh.getParam("gc_topic", mav_topic);
 mav_sub = nh.subscribe<mavros_msgs::Mavlink>(mav_topic, 10, &MavControl::mavCallback, this);
 
 wpm = new WPManagerClient();
 wp_count=0;
 requested=-1;
 state=IDLE;
 ROS_INFO("STATE IDLE");
  
}

void MavControl::send(){
  MavMessenger::send(out_mmsg);
}

void MavControl::mavCallback(const mavros_msgs::Mavlink::ConstPtr& mav){
  // This field has been removed from the MAVLINK message, we need to know what's the effect.
  // if(mav->fromlcm) return;
  in_mmsg = ros2mav(mav);
  process();
}

mavlink_message_t MavControl::ros2mav(const mavros_msgs::Mavlink::ConstPtr& mav){
 mavlink_message_t mmsg;
 mmsg.len=mav->len;
 mmsg.seq=mav->seq;
 mmsg.sysid=mav->sysid;
 mmsg.compid=mav->compid;
 mmsg.msgid=mav->msgid;
 ROS_DEBUG("Received MAV packet");
 ROS_DEBUG("msgid: %d", mmsg.msgid);
 ROS_DEBUG("sysid: %d", mmsg.sysid);
 ROS_DEBUG("compid: %d", mmsg.compid);
 for (size_t i = 0; i < (mmsg.len + 7) / 8; i++){
   //ROS_INFO_STREAM(i << ":" <<(uint64_t)(mmsg.payload64[i]));
    mmsg.payload64[i]=mav->payload64.at(i);
  }
  return mmsg;
}

void MavControl::sendWaypoint(Waypoint wp){
   mavlink_mission_item_t mwp;
   
   ros::Duration duration = wp.getDuration();
   if(wp.isLocal()){
    geometry_msgs::PoseStamped pose = wp.getPose();
//     mwp.frame=MAV_FRAME_LOCAL_ENU;
//     mwp.x=pose.pose.position.x;
//     mwp.y=pose.pose.position.y;
//     mwp.z=pose.pose.position.z;
    mwp.frame=MAV_FRAME_LOCAL_NED;
    mwp.x=pose.pose.position.y;
    mwp.y=pose.pose.position.x;
    mwp.z=-pose.pose.position.z;
    mwp.param4 = tf::getYaw(pose.pose.orientation);
   }else{
    mwp.frame = MAV_FRAME_GLOBAL;  
    geographic_msgs::GeoPose gp = wp.getGeoPose();
    mwp.x=gp.position.latitude;
    mwp.y=gp.position.longitude;
    mwp.z=gp.position.altitude;
    mwp.param4 = tf::getYaw(gp.orientation);
   }
   
   mwp.param1 = (uint64_t)duration.sec*1000+(uint64_t)duration.nsec/1.0e6;
   mwp.param2 = 1.0;
   mwp.param3 = 0.0;
   
   mwp.command=MAV_CMD_NAV_WAYPOINT;
   mwp.target_system=in_mmsg.sysid;
   mwp.target_component=in_mmsg.compid;
   
   mwp.autocontinue = 1;
   mwp.current=0;
   mwp.seq=wp.getId();
   
   
   if(wpm->getListSize()==1) mwp.current=1; 
   
   
   mavlink_msg_mission_item_encode(SYSTEM_ID, MAV_COMP_ID_MISSIONPLANNER, &out_mmsg, &mwp);
   ROS_INFO("Sending Waypoint %d", mwp.seq	);
   send();
}

void MavControl::sendWPCount(int count){
  mavlink_mission_count_t mav_count;
  mav_count.count=count;
  mav_count.target_component=in_mmsg.compid;
  mav_count.target_system=in_mmsg.sysid;
  mavlink_msg_mission_count_encode(SYSTEM_ID, COMP_ID, &out_mmsg, &mav_count);
  send();
  
}
void MavControl::handleGetWaypointList(){
  ROS_INFO_STREAM("Handle Get Waypoint List");
  wp_vec = wpm->getWaypointVector();
  sendWPCount(wpm->getListSize());
  state=WAIT_FOR_REQUEST;
  ROS_INFO("STATE WAIT FOR REQUEST");
  
}

int MavControl::getCount(){
  mavlink_mission_count_t mission_count;
  mavlink_msg_mission_count_decode(&in_mmsg, &mission_count);
  return mission_count.count;
}

void MavControl::sendWPRequest(int sequence){
  mavlink_mission_request_t request;
  request.seq=sequence;
  request.target_system=in_mmsg.sysid;
  request.target_component=in_mmsg.compid;
  mavlink_msg_mission_request_encode(SYSTEM_ID, COMP_ID, &out_mmsg, &request);
  ROS_INFO("Sending a waypoint request");
  send();
}

void MavControl::sendWPAck(MAV_RESULT type){
  mavlink_mission_ack_t ack;
  ack.target_component=in_mmsg.compid;
  ack.target_system=in_mmsg.sysid;
  ack.type=type;
  mavlink_msg_mission_ack_encode(SYSTEM_ID, COMP_ID, &out_mmsg, &ack);  
  ROS_INFO("Sending Waypoint ACK");
  send();
}

void MavControl::handleClearAll(){
  if(state!=IDLE) {
   ROS_ERROR("Trying to do a CLEAR cmd in the middle of a transaction. Denied"); 
   return;
  }
  wpm->clear();
  sendWPAck(MAV_RESULT_ACCEPTED);
}

void MavControl::handleAck(){
  mavlink_mission_ack_t ack;
  mavlink_msg_mission_ack_decode(&in_mmsg, &ack);
  if(ack.type==MAV_RESULT_ACCEPTED){
    ROS_DEBUG("Good Ack Received");
  }else{
    ROS_WARN("Bad Ack Received");
  }
  state=IDLE;
  ROS_INFO("STATE IDLE");
}

void MavControl::process( ){
  
  switch(in_mmsg.msgid){
    
    //Ground Station request current WP list in the robotic vehicle. 
    //Initiates Read WP protocol. Vehicle sends back the WP count and enters WAIT_FOR_REQUEST mode;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      handleGetWaypointList();
    break;
    
    case MAVLINK_MSG_ID_MISSION_REQUEST:
      handleWPRequest();
    break;
    
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
      handleClearAll();
    break;
   
    //GCS sends WP count. Begins WP transmission protocol
    case MAVLINK_MSG_ID_MISSION_COUNT:
      wp_count = getCount();
      ROS_INFO("Received a Mission Count: %d", wp_count);
      requested=0;
      sendWPRequest(requested);
    break;
    
    case MAVLINK_MSG_ID_MISSION_ACK:
      handleAck();
    break;
    
    case MAVLINK_MSG_ID_MISSION_ITEM:
      if(requested<0){
	ROS_WARN("Not Expecting Any Mission Item, skippping...");
	sendWPAck(MAV_RESULT_DENIED);
	break;
      } // Not expecting any mission item
      if(handleMissionItem(requested)){
	requested++;
	if(requested!=wp_count){sendWPRequest(requested);} // get next waypoint
	else{
	  wpm->setListCompleted();
	  state=IDLE;
	  ROS_INFO("STATE IDLE");
	  requested=-1; wp_count=0; sendWPAck(MAV_RESULT_ACCEPTED);} // end of wp list, send ACK
      }
      else { sendWPAck(MAV_RESULT_FAILED); }
    
    break;
      
    
  }
  
  
  
}
/** GCS Requests Waypoints currently on the Vehicle
 */ 
void MavControl::handleWPRequest(){
  //Must have been put in this state by a previous GET COUNT request
  if(state!=WAIT_FOR_REQUEST) {
    ROS_WARN("Received Request for Waypoint outside of an transaction. Ignoring...");
    return;
  }
  
  mavlink_mission_request_t req;
  mavlink_msg_mission_request_decode(&in_mmsg, &req);
  int requested=req.seq;
  ROS_INFO_STREAM("Handle WP Request: " << requested );
  if(requested < wpm->getListSize()){
    sendWaypoint(wp_vec.at(requested));
  }
  else{
    sendWPAck(MAV_RESULT_FAILED);
  }
  if(++requested==wpm->getListSize()) {
    state=WAIT_FOR_ACK;
    ROS_INFO("STATE WAIT FOR ACK");
  }
}

bool MavControl::handleMissionItem(int requested){
 mavlink_mission_item_t item;
 mavlink_msg_mission_item_decode(&in_mmsg, &item);
 
 
 if(item.seq != requested) return false;
 ROS_WARN("Command = %d", item.command);
 switch(item.command){
   case(MAV_CMD_NAV_WAYPOINT):
     Waypoint *wp;
     ROS_WARN("Frame =%d", item.frame);
     if(item.frame==MAV_FRAME_GLOBAL){
      geographic_msgs::GeoPose gp;
      
      //GPS Position
      gp.position.latitude = item.x;
      gp.position.longitude = item.y;
      gp.position.altitude = item.z;
      
      //Orientation
      tf::Quaternion q;
      q.setRPY(0.0, 0.0, item.param4); //param4 = yaw
      tf::quaternionTFToMsg(q, gp.orientation);
      ROS_INFO("quaternion W: %f", gp.orientation.w);
      
      wp = new Waypoint(gp, item.seq);
      ROS_INFO("--WP w : %f", wp->getPose().pose.orientation.w);
    }else if(item.frame==MAV_FRAME_LOCAL_NED){
      ROS_WARN("Local Waypoint");
      wp= new Waypoint(item.y, item.x, -item.z, item.param4, "map", item.seq);  
    }else if(item.frame==MAV_FRAME_LOCAL_ENU){
      ROS_WARN("Local Waypoint");
      wp= new Waypoint(item.x, item.y, item.z, item.param4, "map", item.seq);  
    }
    else{
     return false; 
    }
    wp->setDuration(item.param1);
    ROS_INFO("WP w: %f", wp->getPose().pose.orientation.w);
    wpm->push(*wp);
    break;
   
   default:
    break; 
  }
  return true;
}
// void MavControl::handleWPCommand(){
//   
//  ROS_INFO("Received Waypoint Command");
//  mavlink_command_int_t command;
//  mavlink_msg_command_int_decode(&in_mmsg, &command);
//      
//      
//  switch(command.command){
//    case(MAV_CMD_NAV_WAYPOINT):
// 	 
//     if(command.frame==MAV_FRAME_GLOBAL){
//       geographic_msgs::GeoPoint gp;
//       geodesy::UTMPoint utm;
//       gp.latitude = command.x/10e7;
//       gp.longitude = command.y/10e7;
//       gp.altitude = command.z;
// 	    
//       //geodesy::fromMsg(gp, utm);
//       Waypoint wp(gp, wp_id++);
//       wp.setDuration(command.param1);
//       wpm->push(wp);
//     
//     }
//     if(command.frame==MAV_FRAME_LOCAL_ENU){
//       Waypoint wp(command.x/10e4, command.y/10e4, command.z/10e4, command.param4, "map", wp_id++);
//       wp.setDuration(command.param1);
//       wpm->push(wp);
//     }
//    break;
//    default:  
//    break;
//  }
//      
//       
// }
int main(int argc, char ** argv){
 ros::init(argc, argv, "control");
 MavControl mc;
 
 ros::spin();
 return 1; 
}