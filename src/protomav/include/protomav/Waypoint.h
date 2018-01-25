#ifndef WAYPOINT_HEADER
#define WAYPOINT_HEADER

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geographic_msgs/GeoPose.h>
#include <geodesy/utm.h>

//using namespace geometry_msgs;
class Waypoint{
public:
  Waypoint(geometry_msgs::PoseStamped  pose, ros::Duration duration, int id, float acceptance); //to pushService
  Waypoint(geometry_msgs::PoseStamped  pose, int id);
  Waypoint(double x , double y, double z, double th, std::string frame, int id);
  Waypoint(double x, double y, double z, std::string frame, int id);
  Waypoint(geographic_msgs::GeoPose gp, int id);
  Waypoint(const Waypoint& wp);
  void setFrame(std::string str);
  geometry_msgs::PoseStamped getPose();
  ros::Duration getDuration();
  void setDuration(double d); // if d > epsilon: hovering mode = true
  void setPathType(int trayectory); // Trayectory type [linear:0 quadratic:1 splines:2]
  void setNeighborhood(float nhood);
  void setSpeed(float speed);

  int getId();
  bool isLocal();
  geographic_msgs::GeoPose getGeoPose();
  void init(double x, double y, double z, double th, std::string frame, int id);

  void setAcceptance(float acceptance);
  float getAcceptance();
  //bool reached; // Status
  //bool getStatus();

private:
  geographic_msgs::GeoPose gp_; //
  geometry_msgs::PoseStamped pose_; //
  ros::Duration duration_; // Hold (loiter phase) time  in decimal seconds.
  bool local_frame_; // MAV_FRAME???
  int id_; // WP ID givem by vector position
  // TODO: add to Construtor
  int interpolation_; // Trayectory type [linear:0 quadratic:1 splines:2]
  float speed_; // Scalar quantity of velocity vector over the WP_i
  //float upper_bound;
  //float lower_bound;
  float acceptance_; // Acceptance radius in meters
};




#endif

Waypoint::Waypoint(const Waypoint & wp){
 this->pose_ = wp.pose_;
 this->duration_ = wp.duration_;
 this->id_ = wp.id_;
 this->gp_ = wp.gp_;
 //this->interpolation_ = wp.interpolation_;
 this->local_frame_ = wp.local_frame_;
 this->acceptance_ = wp.acceptance_;
}

void Waypoint::init(double x, double y, double z, double th, std::string frame, int id){
  //PoseStamped * p = new PoseStamped();
 pose_.header.frame_id = frame;
 pose_.header.stamp = ros::Time::now();

 pose_.pose.position.x = x;
 pose_.pose.position.y = y;
 pose_.pose.position.z = z;
 tf::Quaternion q = tf::createQuaternionFromYaw(th);

 ROS_WARN("Setou posicao");
 //Double-check to see if it works
 tf::quaternionTFToMsg(q, pose_.pose.orientation);

 ROS_INFO("Position: %f", getPose().pose.position.x);
 ROS_INFO("Pose: %f", getPose().pose.orientation.w);
 ROS_WARN("Setou pose");
 duration_ = ros::Duration(0.0); //default loither time
 id_ = id;
 acceptance_ = 1.0; // default acceptance radius
}

Waypoint::Waypoint(geometry_msgs::PoseStamped pose,ros::Duration duration, int id, float acceptance):pose_(pose), duration_(duration), id_(id), acceptance_(acceptance),local_frame_(true){}

Waypoint::Waypoint(geometry_msgs::PoseStamped pose, int id):pose_(pose),  duration_(0.0), id_(id), local_frame_(true) ,acceptance_(1.0){}



Waypoint::Waypoint(double x, double y, double z, double th, std::string frame, int id): local_frame_(true){
  init(x,y,z,th, frame, id);
}

Waypoint::Waypoint(double x, double y, double z, std::string frame, int id): local_frame_(true){
  ROS_INFO("Construtor WP sem orientacao");
  init(x, y, z, 0.0, frame, id);
}

Waypoint::Waypoint(geographic_msgs::GeoPose gp, int id):gp_(gp), id_(id),local_frame_(false){
  geodesy::UTMPoint utm;
  geographic_msgs::GeoPoint gpoint = gp.position;
  geodesy::fromMsg(gpoint, utm);

  std::string waypoint_frame;
  ros::param::param<std::string>("waypoint_frame", waypoint_frame, "map");
  ROS_INFO("Criando Waypoint baseado em GPS");

  init(utm.easting, utm.northing, utm.altitude,0.0, waypoint_frame, id);
}

void Waypoint::setFrame(std::string frame){
 pose_.header.frame_id = frame;
}

geometry_msgs::PoseStamped Waypoint::getPose(){
  return pose_;
}

ros::Duration Waypoint::getDuration(){
 return duration_;
}

void Waypoint::setDuration(double d){
 duration_ = ros::Duration(d);
}

void Waypoint::setPathType(int trayectory){
  interpolation_ = trayectory;
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

void Waypoint::setAcceptance(float acceptance){
  acceptance_ = acceptance;
}
float Waypoint::getAcceptance(){
  return acceptance_;
}
