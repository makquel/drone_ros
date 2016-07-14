#ifndef WAYPOINT_HEADER
#define WAYPOINT_HEADER

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geographic_msgs/GeoPose.h>
#include <geodesy/utm.h>

using namespace geometry_msgs;
class Waypoint{
public:
  Waypoint(PoseStamped  pose, ros::Duration duration, int id);
  Waypoint(PoseStamped  pose, int id);
  Waypoint(double x , double y, double z, double th, std::string frame, int id);
  Waypoint(double x, double y, double z, std::string frame, int id);
  Waypoint(geographic_msgs::GeoPose gp, int id);
  Waypoint(const Waypoint& wp);
  void setFrame(std::string str);
  PoseStamped getPose();
  ros::Duration getDuration();
  void setDuration(double d);
  int getId();
  bool isLocal();
  geographic_msgs::GeoPose getGeoPose();
  void init(double x, double y, double z, double th, std::string frame, int id);
private:
  geographic_msgs::GeoPose gp_;
  PoseStamped pose_;
  ros::Duration duration_;
  bool local_frame_;
  int id_;//loitering time

  
};




#endif

Waypoint::Waypoint(const Waypoint & wp){
 this->pose_=wp.pose_;
 this->duration_=wp.duration_;
 this->id_=wp.id_;
 this->gp_=wp.gp_;
 this->local_frame_=wp.local_frame_;
}

void Waypoint::init(double x, double y, double z, double th, std::string frame, int id){
  //PoseStamped * p = new PoseStamped();
 pose_.header.frame_id=frame;
 pose_.header.stamp = ros::Time::now();
 
 pose_.pose.position.x=x;
 pose_.pose.position.y=y;
 pose_.pose.position.z=z;
 tf::Quaternion q= tf::createQuaternionFromYaw(th);
 
 ROS_WARN("Setou posicao");
 //Double-check to see if it works
 tf::quaternionTFToMsg(q, pose_.pose.orientation);
 
 
 ROS_INFO("Pose: %f", getPose().pose.orientation.w);
 ROS_WARN("Setou pose");
 duration_=ros::Duration(0.0);
 id_=id; 
}

Waypoint::Waypoint(PoseStamped pose,  ros::Duration duration, int id):pose_(pose), duration_(duration), id_(id), local_frame_(true){}

Waypoint::Waypoint(PoseStamped pose, int id):pose_(pose),  duration_(0.0), id_(id), local_frame_(true) {}



Waypoint::Waypoint(double x, double y, double z, double th, std::string frame, int id): local_frame_(true){
  init(x,y,z,th, frame, id);
}

Waypoint::Waypoint(double x, double y, double z, std::string frame, int id): local_frame_(true){
  ROS_INFO("Construtor WP sem orientacao");
  init(x, y, z, 0.0, frame, id);
}

Waypoint::Waypoint(geographic_msgs::GeoPose gp, int id):gp_(gp), id_(id),local_frame_(false){
  geodesy::UTMPoint utm;
  geographic_msgs::GeoPoint gpoint=gp.position;
  geodesy::fromMsg(gpoint, utm);
  
  std::string waypoint_frame;
  ros::param::param<std::string>("waypoint_frame", waypoint_frame, "map");
  ROS_INFO("Criando Waypoint baseado em GPS");
  
  init(utm.easting, utm.northing, utm.altitude,0.0, waypoint_frame, id);
}

void Waypoint::setFrame(std::string frame){
 pose_.header.frame_id=frame; 
}

PoseStamped Waypoint::getPose(){
  return pose_;
}

ros::Duration Waypoint::getDuration(){
 return duration_; 
}
  
void Waypoint::setDuration(double d){
 duration_=ros::Duration(d); 
}
int Waypoint::getId(){
  return id_;
}
geographic_msgs::GeoPose Waypoint::getGeoPose(){
 return gp_; 
}

bool Waypoint::isLocal(){
  return local_frame_;
}
