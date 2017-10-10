#include "MavCamera.h"

using namespace protomav;

MavCamera::MavCamera(int width, int height):width_(width),height_(height){
  ros::NodeHandle nh;
  std::string image_topic;
  nh.getParam("image_topic", image_topic);
  image_sub = nh.subscribe<sensor_msgs::CompressedImage>(image_topic, 10, &MavCamera::imageCallback, this);
}

void MavCamera::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& img){
constexpr size_t PAYLOAD_SIZE = sizeof(mavlink_encapsulated_data_t::data);
	cv_bridge::CvImageConstPtr cv_ptr;

	try {
		if (enc::isColor(img_msg->encoding))
			cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
		else
			cv_ptr = cv_bridge::toCvShare(img_msg);

		// code from image_transport_plugins compressed_publisher.cpp
		std::vector<int> params;
		std::vector<uint8_t> jpeg_buffer;
		params.resize(3, 0);
		// typical image size 60-70 packet
		jpeg_buffer.reserve(80 * PAYLOAD_SIZE);

		params[0] = CV_IMWRITE_JPEG_QUALITY;
		params[1] = jpeg_quality;

		if (cv::imencode(".jpg", cv_ptr->image, jpeg_buffer, params)) {
			float comp_ratio = (float)(cv_ptr->image.rows *
					cv_ptr->image.cols *
					cv_ptr->image.elemSize())
				/ jpeg_buffer.size();

			ROS_DEBUG("IMG: JPEG quality %d, ratio %f", jpeg_quality, comp_ratio);

			send_jpeg_image(jpeg_buffer, jpeg_quality,
					cv_ptr->image.rows,
					cv_ptr->image.cols);
		}
		else {
			ROS_ERROR("IMG: cv::imencode (jpeg) failed");
			return;
		}
	}
	catch (cv_bridge::Exception &ex) {
		ROS_ERROR("IMG: %s", ex.what());
	}
	catch (cv::Exception &ex) {
		ROS_ERROR("IMG: %s", ex.what());
	}
}
  

void MavCamera::send_chunk(){
  mavlink_message_t mmsg;
  mavlink_msg_encapsulated_data_encode(system_id, comp_id, &mmsg, &chunk);
  MavMessenger::send(mmsg);
} 

void MavCamera::send_ack(){
  mavlink_message_t mmsg;
  ack.size = image.data.size();
  ack.width = width_;
  ack.height = height_;
  ack.packets = image.data.size()/CHUNK_SIZE+1;
  ack.type = MAVLINK_TYPE_UINT8_T ;
  ack.payload = CHUNK_SIZE;
  ack.jpg_quality = 100;
  mavlink_msg_data_transmission_handshake_encode(system_id, comp_id, &mmsg, &ack);
}


void MavCamera::send(){
 
  
  int char_left = image.data.size();
  for(int i=0; char_left>0; i++){ // enquanto ainda tiver caracteres
    chunk.seqnr=i;
    for(int j=0; j<CHUNK_SIZE && j<char_left; j++){
      chunk.data[j] = image.data.at(CHUNK_SIZE*i+j);
    }
    send_chunk();
    char_left-=253;
  }
  
}

void MavCamera::start(){
  stop_=false;
  if(image.format.compare("jpeg")!=0){
   ROS_ERROR("Compressed Image not in JPEG Format. Image not sent"); 
  }else { send_ack(); }
  while(!stop_){ send();}
}

void MavCamera::stop(){
  stop_=true;
}

void MavCamera::send_jpeg_image(std::vector<uint8_t> &jpeg_buffer, int jpeg_quality,
		int height, int width)
{
	constexpr size_t PAYLOAD_SIZE = sizeof(mavlink_encapsulated_data_t::data);
	mavlink_message_t msg;

	if (jpeg_buffer.empty()) {
		ROS_ERROR("IMG: Empty JPEG buffer!");
		return;
	}

	size_t packet_count = jpeg_buffer.size() / PAYLOAD_SIZE + 1;
	if (jpeg_buffer.capacity() < packet_count * PAYLOAD_SIZE) {
		// preventing copying unowned data in next step
		ROS_DEBUG("IMG: Reserved: %zu -> %zu bytes", jpeg_buffer.capacity(),
				packet_count * PAYLOAD_SIZE);
		jpeg_buffer.reserve(packet_count * PAYLOAD_SIZE);
	}

	ROS_DEBUG("IMG: Send image %d x %d, %zu bytes in %zu packets",
			width, height, jpeg_buffer.size(), packet_count);

	mavlink_msg_data_transmission_handshake_pack_chan(
		gcs_link->get_system_id(),
		gcs_link->get_component_id(),
		gcs_link->get_channel(),
		&msg,
		MAVLINK_DATA_STREAM_IMG_JPEG,
		jpeg_buffer.size(),
		width,
		height,
		packet_count,
		PAYLOAD_SIZE,
		jpeg_quality);
	MavMessenger::send(msg);

	for (size_t seqnr = 0; seqnr < packet_count; seqnr++) {
		mavlink_msg_encapsulated_data_pack_chan(
				gcs_link->get_system_id(),
				gcs_link->get_component_id(),
				gcs_link->get_channel(),
				&msg,
				seqnr,
				jpeg_buffer.data() + (PAYLOAD_SIZE * seqnr));
		
		MavMessenger::send(msg);
		//ROS_DEBUG("IMG: chunk %2zu, %p->%p", seqnr, jpeg_buffer.data(),
		//		jpeg_buffer.data() + (PAYLOAD_SIZE * seqnr));
	}
}




int main(int argc,  char** argv) {
 ros::init(argc, argv, "mav_heartbeat");

 MavCamera camera(480,340);
 camera.start();
 camera.stop();
 
 
}

