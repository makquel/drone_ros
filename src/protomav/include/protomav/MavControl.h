#ifndef MAV_CONTROL_HEADER
#define MAV_CONTROL_HEADER

#include <mavros_msgs/Mavlink.h>
#include <mavlink/v1.0/common/mavlink.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "vehicle.h"
#include <geographic_msgs/GeoPose.h>
#include <geodesy/utm.h>
#include <tf/tf.h>
#include "MavMessenger.h"
#include "WPManagerClient.h"

typedef enum CONTROL_STATE{
  IDLE=0,
  WAIT_FOR_REQUEST=1,
  WAIT_FOR_WAYPOINT=2,
  WAIT_FOR_ACK=3

}CONTROL_STATE;

namespace protomav{

  class MavControl: public MavMessenger{
  public:
    MavControl();
    void mavCallback(const mavros_msgs::Mavlink::ConstPtr& mav);
    mavlink_message_t ros2mav(const mavros_msgs::Mavlink::ConstPtr& mav);
    void handleGetWaypointList();


    int getCount();

    void process();
    bool handleMissionItem(int requested);
//     void handleWPCommand();
    void handleWPRequest();
    void handleClearAll();
    void handleAck();

    void sendWaypoint(Waypoint wp);
    void sendWPRequest(int sequence);
    void sendWPAck(MAV_RESULT type);
    void sendWPCount(int count);
    void send();
    //TODO method for MISSION_ITEM_REACHED ( #46 )
  private:
    WPManagerClient * wpm;
    ros::Subscriber mav_sub;
    int system_id, comp_id;
    mavlink_message_t in_mmsg, out_mmsg;
    int wp_id;
    int wp_count;
    int requested;
    CONTROL_STATE state;
    std::vector<Waypoint> wp_vec;
    //ros::ServiceClient addWP_client;
  };

}
#endif
