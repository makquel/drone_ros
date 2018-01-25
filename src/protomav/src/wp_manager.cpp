#include "WPManager.h"
using namespace protomav;
WPManager::WPManager(std::string wp_frame, std::string planner_frame):wp_frame_(wp_frame), planner_frame_(planner_frame),new_wp(false), current(0)
{

  ros::NodeHandle nh;
  WPadd = nh.advertiseService("add_waypoint", &WPManager::pushService, this);
  WPsize = nh.advertiseService("get_wp_list_size", &WPManager::getListSizeService, this);
  WPlist = nh.advertiseService("get_wp_list", &WPManager::getWPVectorService, this);
  WPclear = nh.advertiseService("clear_wp_list", &WPManager::clearService, this);
  WPcompleted = nh.advertiseService("set_list_completed", &WPManager::setListCompletedService, this);

  //mb_client = new MoveBaseClient("move_base",false);
  //nh.getParam("altitude_topic", altitude_topic);
  //nh.getParam("gps_topic", gps_topic);
  gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("/xsens/gps_data", 10, &WPManager::gpsCallback, this);// Use Yaml config file
  alt_sub = nh.subscribe<sf11_altimeter::sensor_data>("/altitude", 10, &WPManager::altCallback, this);
  balt_sub = nh.subscribe<sensor_msgs::FluidPressure>("/xsens/pressure", 10, &WPManager::baltCallback, this);
}

int WPManager::getListSize(){
  return wp_list.size();
}

std::vector<Waypoint> WPManager::getWaypointList(){
  std::vector<Waypoint> ret = wp_list;
  return ret;
}

bool WPManager::WPtoMsg(Waypoint & wp, protomav::WaypointMsg & wp_msg){
  //TODO Fill up with all the required paramters
  wp_msg.pose = wp.getPose();
  wp_msg.loiter = wp.getDuration();
  wp_msg.id = wp.getId();
  wp_msg.acceptance = wp.getAcceptance();
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
    Waypoint wp(req.waypoint.pose, req.waypoint.loiter, req.waypoint.id, req.waypoint.acceptance);
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
void WPManager::setStatus(bool state){
  reached = state;
}
bool WPManager::getStatus(){
  return reached;
}
void WPManager::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &gps){

  gps_latitude = gps->latitude*1.0e7;
  gpoint.latitude = gps->latitude;
  gps_longitude = gps->longitude*1.0e7;
  gpoint.longitude = gps->longitude;
  geodesy::fromMsg(gpoint, utm);
}
// The following is a huge gambiarra
// You'd try use mutex and a process to mix'em up!
// http://answers.ros.org/question/12421/subscribing-2-topics-of-different-types-in-a-callback-function/
// or
// Use timeSync
// http://answers.ros.org/question/47406/callback-for-two-topics-at-the-same-time/
void WPManager::altCallback(const sf11_altimeter::sensor_data::ConstPtr& alt){
  //gpoint.altitude = alt->altitude;
}

void WPManager::baltCallback(const sensor_msgs::FluidPressure::ConstPtr& pressure){
  float cte = 145366.45;
  float expo = 0.190284;
  gpoint.altitude = pow((1-(pressure->fluid_pressure/1013.25)),expo)*cte;
}

double WPManager::distance (geometry_msgs::PoseStamped &curr_wp, geodesy::UTMPoint &curr_pose){
  ///
  /// abscissa: UTMPoint.easting [East to West]
  /// ordinate: UTMPoint.northing [North to South]
  /// zone: UTMPoint.zone [Grid position]
  //TODO:   include altitude in the distance computation
  double x = curr_wp.pose.position.x - curr_pose.easting;
	double y = curr_wp.pose.position.y - curr_pose.northing;
	double dist;

	dist = pow(x, 2) + pow(y, 2);
	dist = sqrt(dist);

	return dist;
}

void WPManager::run(){
 tf::TransformListener listener;
 if(!new_wp) return;
 //ROS_INFO("Waypoint List Size = %d", getListSize());
 move_base_msgs::MoveBaseGoal goal;
 Waypoint * wp;
 for(; current<getListSize(); current++){
   wp = &(wp_list.at(current));
   geometry_msgs::PoseStamped wp_pose;
   setStatus(false);
   // TODO: Should be a Switch
   if(wp->isLocal()){
     wp_pose = wp->getPose(); //
     ROS_INFO("Current Waypoint East: %f", wp_pose.pose.position.x);
     ROS_INFO("Current Waypoint North: %f", wp_pose.pose.position.y);
     ROS_INFO("Current Waypoint Down: %f", wp_pose.pose.position.z);
     //ROS_INFO("Orientation(w): %f", wp_pose.pose.orientation.w);
   }
   else{
     ROS_INFO("Opss a daisy...");
     /* TODO*/
   }

  /*try{
   listener.transformPose(planner_frame_, wp_pose, goal_pose);
  }
  catch(tf::TransformException ex){
   ROS_ERROR("%s", ex.what());
   continue;
 }*/

  broadcast.current(current);
  //TODO: Send goal to path following algorithm

  float acceptance = wp->getAcceptance();
  ROS_WARN("Acceptance radius:= %f", acceptance);
  while (!reached){
    ///current_pose.position.longitude = 0.0;
    //ROS_INFO("Current x: %f", utm.easting);
    //ROS_INFO("Current y: %f", utm.northing);
    double dist = distance(wp_pose, utm);

    if(dist < 40.0){
          ROS_INFO_THROTTLE(1,"D:= %f", dist);
          //ROS_INFO("Current x: %f", utm.easting);
          //ROS_INFO("Current y: %f", utm.northing);
          ROS_INFO_THROTTLE(1,"Current z: %f", utm.altitude);
    }
    if(dist < acceptance){
      ROS_INFO("***Waypoint reached***"); // Is it enough to enough to guarantee??
      broadcast.reached(current);
      setStatus(true);
    }
    ros::spinOnce();
  }
  if(current == getListSize()-1){
    ROS_WARN("Last wp");
  }else{
    ROS_WARN("On its way to next wp");
  }
  //TODO: what if the vehicle doesn't reached its goal???
  /*if(mb_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("Waypoint reached");
    broadcast.reached(current);
  }
  else{
    ROS_INFO("The robot/vehicle wasn't able to reach the waypoint. Going for the next one");
    continue;
  }*/

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
