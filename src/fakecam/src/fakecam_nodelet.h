/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FAKECAM_NODELET_H
#define FAKECAM_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <stdio.h>
#include <sstream> // string stream
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <cv_bridge/cv_bridge.h>
#include <math.h>       /* round, floor, ceil, trunc, fabs */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>



namespace fakecam
{

/// all atributes have a "_" suffix to differentiate them from local variables
class FakecamNodelet : public nodelet::Nodelet
{
public:
  FakecamNodelet()  { }

  ~FakecamNodelet() {  }// destructor

  void callback(const ros::TimerEvent& e);
  void RawImgCallback(const sensor_msgs::ImageConstPtr& image_msg);
  const char* WINDOW;/// used for debug.

private:
  virtual void onInit();
    bool initHasRun; /// need to know if init has ended.

  // variables to calculate frame rate
  ros::Time lastTStamp_;
  unsigned int lastFrameCount_;
  unsigned int frameCount_;
  double fps_;
  double framerate_; /// desired framerate

  int printIntervalSec_; /// print status message every X seconds

  //image_transport::ImageTransport* it;
  boost::shared_ptr<image_transport::ImageTransport> it;

  image_transport::Publisher pub_;
  sensor_msgs::ImagePtr image_msg_;

  cv_bridge::CvImage bridgedImage_; // pre-alocate output image

  ros::Timer timer1_;/// we are fake, so our "camera driver" is only a timer to generate a periodic callback...
  char current_color_;

  image_transport::Subscriber image_sub_; // get images from camera
  image_transport::Publisher imageSlow_pub_; // uncomment if you want to publish

  /// used to publish
  std::string slower_image_topic_;
  int pub_every_frame_;
  int count_frames_;

  int width_;
  int height_;

}; // class

}//namespace


#endif
