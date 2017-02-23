#include <inlude/mtig_nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <nodelet_topic_tools/nodelet_throttle.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(xsens_driver::XsensNodelet, nodelet::Nodelet);

namespace xsens_driver
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
  void XsensNodelet::onInit()
  {
    // a private nodehandle create a fakecam namespace for parameters and topics
    ros::NodeHandle private_nh = getMTPrivateNodeHandle();
    // images must be sent though an image transport
    // this provides compression magic!! (theora and JPEG)
    image_transport::ImageTransport it(private_nh); // init ImageTransport
    // we will publich a topic called image
    // it is not a creative name: let the user remap it if he wants!
    pub_ = it.advertise("image", 1);
    
    // check if there is a parameter framerate in private namespace
    // in other works, check for a /fakecam/framerate parameter
    // if there is not a parameter, assume the default value given
    private_nh.param("framerate", framerate_ ,30.0);
    //ROS also has a nice ASSERT macro.
    //if the user set a negative framerate, shows a message and kills the process.
    // useful to make sure shit will not happen
    ROS_ASSERT_MSG(framerate_>0,"onInit: framerate can not be negative, it is %.2f",framerate_);
    
    // message tells the user about the intended framerate
    NODELET_INFO("onInit: will set framerate to %.1f",framerate_);
    
     
    // imageptr is a boost shared pointer, this is the recipe to initialize it
    // and a shared pointer is part of the image transport recipe!
    image_msg_ = boost::shared_ptr<sensor_msgs::Image>(new sensor_msgs::Image);
    
    int aux; // aux is needed due to type conflicts with image_msg_ fields.
    private_nh.param("height", aux ,480);
    image_msg_->height = aux;
    private_nh.param("width" , aux  ,680);
    image_msg_->width = aux;
    
    image_msg_->data.resize(image_msg_->height*image_msg_->width);// pre-allocated size
    
    frameCount_=0;
    printIntervalSec_ = 5; ///default interval to print messages
    fps_ = 0;
    current_color_ = 128; /// grey 50;
    
    // if we had a real driver, we would setup a camera library
    // and setup the call back function to run when the camera has a new image.
    // but this is fake, so we only setup a timer...
    // also note the syntax to define a object method as a callback
    timer1_ = private_nh.createTimer(ros::Duration(1.0/framerate_)
                    , &FakecamNodelet::callback, this);
    
  }//onInit
  
  
void XsensNodelet::callback(const ros::TimerEvent& e) {
    
  if (!ros::ok()) { //check if ros is shutting down
        // if this were a real driver, we could shut stuff down here.
        return; /// ros is shutting down, do nothing.
  }
  frameCount_++; // count how many frames we have sent
    
  // at the first frame, save its time and count
  if (1 == frameCount_) { lastTStamp_ = ros::Time::now(); lastFrameCount_ = 1; }
  
  // note: a type for time interval and other time to clock.
  // interval <-- clocknow - clockformer
  ros::Duration elapsed = ros::Time::now() - lastTStamp_;
  
  // check if it is time to print out a message
  if ( elapsed.toSec() > printIntervalSec_ ) {
    // calculate the real fps to show the user!
    fps_ = (frameCount_ - lastFrameCount_) / elapsed.toSec();
    // save the current as the last for the next iteration
    lastFrameCount_ = frameCount_;
    lastTStamp_ = ros::Time::now();
  }
  
  
  // a message to ros console, the argument syntax is printf-like
  // the THROTTLE macro means it will only actually send a message 
  // each printIntervalSec seconds. Avoid too much messages.
  NODELET_INFO_THROTTLE(printIntervalSec_,"fakecam: %d frames fps %.2f",frameCount_,fps_);
  
  
    // the header has a timestamp. fill it with NOW!!!
    image_msg_->header.stamp = ros::Time::now();
    
    
    // encoding: MONO8 is one byte per byte greyscale
    // http://docs.ros.org/electric/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html
    image_msg_->encoding = sensor_msgs::image_encodings::MONO8; 
    // how many bytes to skip to go to the next line
    // no, they are not always the same. it depends on the encoding!
    image_msg_->step = image_msg_->width ;
    
    // all we do is to fill the image with a uniform grey, changing at each frame
    memset (&image_msg_->data[0],current_color_++,
            image_msg_->height*image_msg_->width);
    
    //the header also contains a counter. 
    image_msg_->header.seq = frameCount_;
    // mama, I publish a message!
    pub_.publish(image_msg_);
    
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