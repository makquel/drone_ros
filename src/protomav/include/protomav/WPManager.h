#ifndef WPMANAGER_HEADER
#define WPMANAGER_HEADER
#include "Waypoint.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "protomav/AddWaypoint.h"
#include "protomav/GetWPListSize.h"
#include "protomav/GetWPVector.h"
#include "protomav/ClearWaypoints.h"
#include "protomav/SetListCompleted.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "MavBroadcast.h"
// #include <geodesy/UTMPose.h>
namespace protomav {
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  class WPManager{
  private:
    //std::list<Waypoint> wp_list;
    std::vector<Waypoint> wp_list, wp_list_temp;
    std::set<int> id_set;
    std::string wp_frame_;
    std::string planner_frame_;
    ros::ServiceServer WPadd, WPsize, WPlist, WPclear, WPcompleted;
    MoveBaseClient * mb_client;
    bool new_wp;
    int current;
    MavBroadcast broadcast;
    
  public:  
    WPManager(std::string wp_frame, std::string planner_frame);
    bool push(Waypoint wp);
    bool pushService(protomav::AddWaypoint::Request &req, protomav::AddWaypoint::Response & res);
    bool getWPVectorService(protomav::GetWPVector::Request &req, protomav::GetWPVector::Response &res);
    bool getListSizeService(protomav::GetWPListSize::Request &req, protomav::GetWPListSize::Response &res);
    bool setListCompletedService(protomav::SetListCompleted::Request &req, protomav::SetListCompleted::Response &res);
  // Waypoint *pop();
    void run();
    int getListSize();
    //std::list<Waypoint>  getWaypointList();
    std::vector<Waypoint> getWaypointList();
    bool WPtoMsg(Waypoint & wp, protomav::WaypointMsg & wp_msg);
    bool clear();
    bool clearService(protomav::ClearWaypoints::Request &req, protomav::ClearWaypoints::Response &res);
    void setListCompleted();  //geodesy::UTMPose getOrigin();
    int getCurrent();
    void setCurrent(int c);
    //void setOrigin(geodesy::UTMPose p);
    
  };

}
#endif