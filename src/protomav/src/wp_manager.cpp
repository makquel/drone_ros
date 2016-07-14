#include "WPManager.h"
using namespace protomav;
WPManager::WPManager(std::string wp_frame, std::string planner_frame):wp_frame_(wp_frame), planner_frame_(planner_frame),new_wp(false), current(0)
{
  ros::NodeHandle nh;
  WPadd=nh.advertiseService("add_waypoint", &WPManager::pushService, this);
  WPsize=nh.advertiseService("get_wp_list_size", &WPManager::getListSizeService, this);
  WPlist=nh.advertiseService("get_wp_list", &WPManager::getWPVectorService, this);
  WPclear=nh.advertiseService("clear_wp_list", &WPManager::clearService, this);
  WPcompleted=nh.advertiseService("set_list_completed", &WPManager::setListCompletedService, this);
  mb_client = new MoveBaseClient("move_base",false);
  
}

int WPManager::getListSize(){
  return wp_list.size();
}

std::vector<Waypoint> WPManager::getWaypointList(){
  std::vector<Waypoint> ret = wp_list;
  return ret;
}

bool WPManager::WPtoMsg(Waypoint & wp, protomav::WaypointMsg & wp_msg){
 wp_msg.pose=wp.getPose();
 wp_msg.loiter=wp.getDuration();
 wp_msg.id=wp.getId();
}

bool WPManager::getWPVectorService(protomav::GetWPVector::Request &req, protomav::GetWPVector::Response &res){
    std::vector<Waypoint> wp_vector = getWaypointList();
    std::vector<protomav::WaypointMsg> wp_msg_vector;
    
    for (std::vector<Waypoint>::iterator it = wp_vector.begin() ; it != wp_vector.end(); ++it){ 
      protomav::WaypointMsg wp_msg;
      WPtoMsg(*it, wp_msg);
      wp_msg_vector.push_back(wp_msg);
    }
    res.wp_vector=wp_msg_vector;
    return true;
  
}
 
bool WPManager::pushService(AddWaypoint::Request &req,
			    AddWaypoint::Response & res){
    ROS_INFO("Service Called");
    Waypoint wp(req.waypoint.pose, req.waypoint.loiter, req.waypoint.id);
    res.inserted = push(wp);
    return true;
}
bool WPManager::push(Waypoint wp){
  std::pair<std::set<int>::iterator,bool> ret;
  ret = id_set.insert(wp.getId());
  
  if(ret.second==false) {
    ROS_INFO("Waypoint ID already in set, not inserting");
    return false;
  }
  wp_list_temp.push_back(wp); 
  ROS_INFO("Waypoint added to the waypoint list");
  return true;
}

bool WPManager::getListSizeService(protomav::GetWPListSize::Request &req, protomav::GetWPListSize::Response &res){
 res.size=getListSize(); 
 return true;
}

bool WPManager::clearService(ClearWaypoints::Request &req, ClearWaypoints::Response & res){
  res.cleared=clear();
  return true;
}

bool WPManager::clear(){
  wp_list.clear();
  setCurrent(0);
  return true;
}

bool WPManager::setListCompletedService(protomav::SetListCompleted::Request &req, protomav::SetListCompleted::Response &res){
    setListCompleted();
    res.completed = true;
    return true;
}

void WPManager::setListCompleted(){
  new_wp=true;
  wp_list=wp_list_temp;
  wp_list_temp.clear();
  id_set.clear();
  ROS_INFO("New Waypoint List Set, size=%d", getListSize());
}


int WPManager::getCurrent(){
  return current;
}

void WPManager::setCurrent(int c){
  current=c;
}

void WPManager::run(){
 tf::TransformListener listener;
 if(!new_wp) return;
 //ROS_INFO("Waypoint List Size = %d", getListSize());
 move_base_msgs::MoveBaseGoal goal;
 Waypoint * wp;
 for(; current<getListSize(); current++){
  wp=&(wp_list.at(current));
  PoseStamped goal_pose, wp_pose;
  if(wp->isLocal()){
    wp_pose=wp->getPose();
    ROS_INFO("Orientation(w): %f", wp_pose.pose.orientation.w);
  }
  else{
    /* TODO*/
  }
  
  try{
   listener.transformPose(planner_frame_, wp_pose, goal_pose);
  }
  catch(tf::TransformException ex){
   ROS_ERROR("%s", ex.what());
   continue;
  }  
  
  
  goal.target_pose=goal_pose;
  goal.target_pose.pose.position.z=0.0;
  broadcast.current(current);
  mb_client->sendGoal(goal);
  
  while(!mb_client->waitForResult(ros::Duration(0.2))){ros::spinOnce();}
  
  if(mb_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Waypoint reached");
    broadcast.reached(current);
  }
  else{
    ROS_INFO("The robot/vehicle wasn't able to reach the waypoint. Going for the next one");
    continue;
  }
 
  ros::Rate r(10.0);
  ros::Time start = ros::Time::now();
  while(ros::ok()){
   ros::Time now = ros::Time::now();
   if(now-start > wp->getDuration()) break;
   ros::spinOnce();
   r.sleep();
  }
 }
 new_wp=false;
}

main(int argc, char**argv){
 ros::init(argc, argv, "wp_manager");
 ros::NodeHandle nh;
 std::string wp_frame, planner_frame;
 nh.getParam("wp_frame", wp_frame);
 nh.getParam("planner_frame", planner_frame);
 WPManager wpm(wp_frame, planner_frame);
 while(ros::ok()){
  wpm.run();
  ros::spinOnce();
 }
 return 0; 
}