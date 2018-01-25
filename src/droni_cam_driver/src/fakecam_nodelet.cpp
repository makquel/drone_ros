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

#include "fakecam_nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/Image.h>
#include <pluginlib/class_list_macros.h>
using namespace cv;

// part of the cake recipe for a nodelet!
// note, this is a plugin (a library) loadable by a manager
// this is how the manager can find this library
// old version! nodelets tutorials do not use the newer plugin lib macros (but this still work)
//PLUGINLIB_DECLARE_CLASS(fakecam, FakecamNodelet, fakecam::FakecamNodelet, nodelet::Nodelet);
// new version of plugin lib. only two parameters
PLUGINLIB_EXPORT_CLASS(fakecam::FakecamNodelet, nodelet::Nodelet);

namespace fakecam
{

// this is to use the throttle nodelet from the nodelet_topic_tools package
// we must declare a nodelet ourselves because it is generic templated class.
// so, we must create a concrete class, and then declare the nodelet.
// note, the declaration is inside the namespace, so the "fakecam::" is implicit
typedef nodelet_topic_tools::NodeletThrottle<sensor_msgs::Image> NodeletThrottleImage;
// this macro has two arguments:
// first: the class which implements the plugin (a class loadable in runtime)
// second: the superclass, the plugin type. In this case, it is a nodelet.
PLUGINLIB_EXPORT_CLASS(fakecam::NodeletThrottleImage, nodelet::Nodelet);



/**
 * \brief this nodelet method is called after the lib is loaded.
 * \par Initialize all you need here. Setup callbacks.
 *
 */
  void FakecamNodelet::onInit()
  {
    ros::NodeHandle nh_ = getMTNodeHandle();
    // a private nodehandle create a fakecam namespace for parameters and topics
    ros::NodeHandle private_nh = getMTPrivateNodeHandle();
    // images must be sent though an image transport
    // this provides compression magic!! (theora and JPEG)
    //image_transport::ImageTransport it(private_nh); // init ImageTransport
    it = boost::make_shared<image_transport::ImageTransport>(private_nh);
    // we will publich a topic called image
    // it is not a creative name: let the user remap it if he wants!
    //pub_ = it.advertise("image", 1);


    // imageptr is a boost shared pointer, this is the recipe to initialize it
    // and a shared pointer is part of the image transport recipe!
    image_msg_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);

    // check if there is a parameter framerate in private namespace
    // in other works, check for a /fakecam/framerate parameter
    // if there is not a parameter, assume the default value given
    //private_nh.param("framerate", framerate_ ,30.0);
    //ROS also has a nice ASSERT macro.
    //if the user set a negative framerate, shows a message and kills the process.
    // useful to make sure shit will not happen
    //ROS_ASSERT_MSG(framerate_>0,"onInit: framerate can not be negative, it is %.2f",framerate_);

    // message tells the user about the intended framerate
    //NODELET_INFO("onInit: will set framerate to %.1f",framerate_);


    // imageptr is a boost shared pointer, this is the recipe to initialize it
    // and a shared pointer is part of the image transport recipe!
    /*image_msg_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);

    int aux; // aux is needed due to type conflicts with image_msg_ fields.
    private_nh.param("height", aux ,480);
    image_msg_->height = aux;
    private_nh.param("width" , aux  ,680);
    image_msg_->width = aux;

    image_msg_->data.resize(image_msg_->height*image_msg_->width);// pre-allocated size

    frameCount_=0;
    printIntervalSec_ = 5; ///default interval to print messages
    fps_ = 0;
    current_color_ = 128; /// grey 50;*/

    // if we had a real driver, we would setup a camera library
    // and setup the call back function to run when the camera has a new image.
    // but this is fake, so we only setup a timer...
    // also note the syntax to define a object method as a callback
    //timer1_ = private_nh.createTimer(ros::Duration(1.0/framerate_), &FakecamNodelet::callback, this);

    nh_.param("resize_width", width_, (int)640);
    nh_.param("resize_height", height_, (int)480);

    // find name of image topic and subscribe
    std::string imagetopicname;
    nh_.param<std::string>("image_topic_name",imagetopicname,"/camera/image_mono");
    // advertise slower image topic, if needed
    private_nh.param("publish_slower_image",pub_every_frame_, 0);
    if (pub_every_frame_) {
     // set name of topic to publish slower images
     slower_image_topic_ = imagetopicname+"_slow";
     imageSlow_pub_ = it->advertise(slower_image_topic_, 1);
   }


    // pre-alocating output image with the correct output size
    // attention: height_ is #rows, width_ is #collumns. rows come first
    bridgedImage_.image = cv::Mat(height_, width_ , CV_8UC1);


    NODELET_INFO("FakecamNodelet::onInit subscribing to %s ", imagetopicname.c_str());
    image_sub_ = it->subscribe(imagetopicname, 1, &FakecamNodelet::RawImgCallback, this);



    initHasRun = true;
    NODELET_INFO("FakecamNodelet::onInit");
  }//onInit



void FakecamNodelet::RawImgCallback(const sensor_msgs::ImageConstPtr& image_msg)
{
  if (!initHasRun) { NODELET_WARN_THROTTLE(1,"fakecam callback: onInit() has not run !!!"); return; }
  NODELET_INFO_THROTTLE(10,"fakecam callback: image %d x %d",image_msg->width, image_msg->height);

  cv::Mat buffer = cv::Mat(image_msg->height, image_msg->width , CV_8UC1);
  cv::Mat image_rot = cv::Mat(image_msg->height, image_msg->width , CV_8UC1);

  //////////// cv bridge, convert ros message in opencv object
  cv_bridge::CvImageConstPtr cv_ptr;
  try  {
    cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);//SHARED!!!
  } catch (cv_bridge::Exception& e) {
    NODELET_ERROR("image call back: cv_bridge exception: %s", e.what());
    return;
  }//try



  if (pub_every_frame_>0) {
    if ((count_frames_ % pub_every_frame_)==0) {

      bridgedImage_.header = image_msg->header;

      bridgedImage_.encoding = sensor_msgs::image_encodings::MONO8;

      //cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg);
      //cv_bridge::CvImagePtr cv_image_scaled = boost::make_shared<cv_bridge::CvImage>();
      //cv_image_scaled->header = cv_image->header;
      //cv_image_scaled->encoding = cv_image->encoding;

      //cv::resize(cv_ptr->image, bridgedImage_.image, cv::Size(width_, height_),0, 0, cv::INTER_LINEAR);
      //bridgedImage_.image = cv_ptr->image;
      buffer = cv_ptr->image;
      cv::resize(buffer, bridgedImage_.image, cv::Size(width_, height_),0, 0, cv::INTER_LINEAR);

      //image_rot = bridgedImage_.image;
      cv::flip(bridgedImage_.image, bridgedImage_.image,-1); //Flips 180
      cv::flip(bridgedImage_.image, bridgedImage_.image,1); // mirror around y axis
      // Draw a rectangle ( 5th argument is not -ve)
      //cv::rectangle( bridgedImage_.image, Point( 15, 20 ), Point( 70, 50), Scalar( 255, 255, 255 ), +1, 4 );
      //cv::imshow("Image1", bridgedImage_.image);
      // here it waits for 1ms, but we can not wait less time.
      //cv::waitKey(1); // needed for imshow http://stackoverflow.com/questions/5217519/opencv-cvwaitkey
      // Draw a filled rectangle ( 5th argument is -ve)
      //rectangle( image, Point( 115, 120 ), Point( 170, 150), Scalar( 100, 155, 25 ), -1, 8 );
      // tried updateWindow instead of waitKey, but got the error below:
      //OpenCV Error: No OpenGL support (The library is compiled without OpenGL support)
      //cv::updateWindow("Image1");
      //cv::resize(cv_image->image, cv_image_scaled->image, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
      //bridgedImage.image = cv_ptr->image;
      imageSlow_pub_.publish(bridgedImage_.toImageMsg());
      count_frames_ = 0;

    }//endif of publish slow image topic
  }
  count_frames_++; // always increment even if it do not publish

  /*  format of the image ros message
  $ rosmsg show sensor_msgs/Image
  Header header
  uint32 seq
  time stamp
  string frame_id
  uint32 height
  uint32 width
  string encoding
  uint8 is_bigendian
  uint32 step
  uint8[] data
  */

}// callback



}//namespace
