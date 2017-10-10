#include "WPManagerClient.h"
using namespace protomav;


WPManagerClient::WPManagerClient(){

  ros::NodeHandle nh;
  add = nh.serviceClient<protomav::AddWaypoint>("add_waypoint");
  size = nh.serviceClient<protomav::GetWPListSize>("get_wp_list_size");
  list= nh.serviceClient<protomav::GetWPVector>("get_wp_list");
  completed = nh.serviceClient<protomav::SetListCompleted>("set_list_completed");
  clearWP = nh.serviceClient<protomav::ClearWaypoints>("clear_wp_list");
}

int WPManagerClient::getListSize(){

  protomav::GetWPListSize srv;
  if(size.call(srv)){
    ROS_INFO("List Size: %d", srv.response.size);
  }
  else{
    ROS_ERROR("Could not get Waypoint list size");
  }
  return srv.response.size;
}

std::vector<Waypoint> WPManagerClient::getWaypointVector(){
  protomav::GetWPVector srv;
  std::vector<Waypoint> res;
  if(list.call(srv)){
   ROS_INFO("Got WP List(vector)");
   std::vector<protomav::WaypointMsg> wp_msg_vector = srv.response.wp_vector;
   for (std::vector<protomav::WaypointMsg>::iterator it = wp_msg_vector.begin() ; it != wp_msg_vector.end(); ++it){
    Waypoint wp((*it).pose, (*it).loiter, (*it).id, (*it).acceptance);
    res.push_back(wp);
   }
  }else{
    ROS_ERROR("Could not get WP List(vector)");
  }


  return res;
}

bool WPManagerClient::clear(){
 protomav::ClearWaypoints srv;

 if(clearWP.call(srv)){
  return srv.response.cleared;
 }
 else{
  ROS_ERROR("Could not clear the waypoint list");
  return false;
 }

}


bool WPManagerClient::setListCompleted(){
    protomav::SetListCompleted srv;
    if(completed.call(srv)){
     return srv.response.completed;
    }
    else{
      ROS_ERROR("Could not end transaction");
      return false;
    }

}
bool WPManagerClient::push(Waypoint wp){
    protomav::AddWaypoint srv;
    ROS_INFO("WP Pose w %f", wp.getPose().pose.orientation.w);
    srv.request.waypoint.pose = wp.getPose();
    srv.request.waypoint.loiter = wp.getDuration();
    srv.request.waypoint.id = wp.getId();
    srv.request.waypoint.acceptance = wp.getAcceptance();
    ROS_INFO("Pose orientation w: %f", srv.request.waypoint.pose.pose.orientation.w);
    if(add.call(srv)){
      ROS_INFO("Waypoint added");
    }else{
      ROS_ERROR("Could not add Waypoint");
    }
}
