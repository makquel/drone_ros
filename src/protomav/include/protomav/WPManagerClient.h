#include "WPManager.h"
namespace protomav{
  class WPManagerClient{
  private:
    ros::ServiceClient add, size, list, clearWP, completed;
  public:
    WPManagerClient();
    int getListSize();
    bool push(Waypoint wp);
    std::vector<Waypoint> getWaypointVector();
    bool clear();
    bool setListCompleted();
  };
}
