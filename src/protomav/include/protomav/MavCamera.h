#ifndef MAV_CAMERA_HEADER
#define MAV_CAMERA_HEADER

#include "MavMessenger.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

#define CHUNK_SIZE 253

namespace protomav{
class MavCamera : public protomav::MavMessenger{
public:
  MavCamera(int width, int height);
  virtual void send();
  void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& img);
  void send_ack();
  void send_chunk();
  void start();
  void stop();

  
private:
  ros::Subscriber image_sub;
  mavlink_encapsulated_data_t chunk;
  sensor_msgs::CompressedImage image;
  mavlink_data_transmission_handshake_t ack;
  bool stop_;
  int width_, height_;
  
};
}

#endif