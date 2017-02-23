#ifndef MTIG_NODELET_H
#define MTIG_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sensor_msgs/image_encodings.h> 
#include <math.h> //fabs
#include <boost/shared_ptr.hpp>

#include "CallbackHandler.h"

namespace xsens_driver
{

/// all atributes have a "_" suffix to differentiate them from local variables 
class XsensNodelet : public nodelet::Nodelet
{
public:
  XsensNodelet()  {}

  ~XsensNodelet() {}// destructor
  
  void callback(const ros::TimerEvent& e);
  
  
private:
  virtual void onInit();

  XsControl* control = XsControl::construct();
  
  // variables to calculate frame rate
  ros::Time lastTStamp_;
  unsigned int lastFrameCount_;
  unsigned int frameCount_;
  double fps_;
  double framerate_; /// desired framerate

  int printIntervalSec_; /// print status message every X seconds
  
  //image_transport::ImageTransport* it;
  image_transport::Publisher pub_;
  sensor_msgs::ImagePtr image_msg_;
  
  ros::Timer timer1_;/// we are fake, so our "camera driver" is only a timer to generate a periodic callback...
  char current_color_;
  

  
}; // class

}//namespace


#endif 