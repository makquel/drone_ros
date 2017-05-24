#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>
//#include <sensor_msgs/NavSatFix.h>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

#include <sf11_altimeter/sensor_data.h>

serial::Serial ser;
std::string serial_port;
int serial_speed;
bool CTS_status;
int sampling_rate;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data.c_str());
    ser.write(msg->data);
}

int main (int argc, char** argv){
    //creating the nodde
    ros::init(argc, argv, "sf11c_lightware_node");
    ros::NodeHandle nh;
    int16_t t_avail = 0;// serial port unavailability

    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher altitude = nh.advertise<sf11_altimeter::sensor_data>("altitude", 5);

    //unsigned char xml_parser_on[] = { 0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0x30, 0xEF, 0xE8, 0xFF, 0xFE, 0x02, 0x44, 0x31, 0x05, 0x72 };
    //std::vector<unsigned char> airspeed_on(xml_parser_on, xml_parser_on+21);

    //setting default device path for the sensor
    nh.param("serial_port", serial_port, std::string("/dev/sensors/sf11")); ///dev/sensors/sf11
    //setting default device communication speed for the sensor
    nh.param("serial_speed", serial_speed, int(115200));
    // setting default sampling rate
    nh.param("sampling_rate", sampling_rate, int(16));
    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(serial_speed);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        ser.setRTS(true);
        ser.setDTR(true);
        //ser.getCTS();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
        //ser.write(airspeed_on);
    }else{
        return -1;
    }


    ros::Duration duration(1./float(sampling_rate)); //0.25s



    std::vector<unsigned char> response;
    duration.sleep(); //Xbee timer after power*up

    while(ros::ok()){
      std_msgs::Float32 result;
      sf11_altimeter::sensor_data msg;
      ros::spinOnce();

        //ser.write(hexstring);
        //duration.sleep();

        //std::cout << "CTS:" << ser.getCTS() << "\n";
        //std::cout << "Buffer size: " << ser.available() << "\n";
        if(ser.available()){
          //ROS_INFO("I'm in the mood!.");
            //ROS_INFO_STREAM("Reading from serial port");
            //std_msgs::Float32 result;
            //sf11_altimeter::sensor_data msg;
            ser.read(response, ser.available());
            //std::cout << "Vector size: " << response.size() << "\n";
            double x;
            for(int i=0; i<response.size(); ++i){
              if(response[i] == 'm'){
                //std::cout << "found it!";
                x = ((response[i-5]- '0')*1 + (response[i-3]- '0')*0.1 + (response[i-2]- '0')*0.01);
              }
            }
            result.data = x;
            msg.altitude = x;
            msg.unavailability = t_avail;
            //msg.header.stamp = ros::Time::now();
            ROS_INFO_STREAM("Altitude[m]: " << result.data);

        }else{
          ROS_ERROR_STREAM("Serial port unavailable" << t_avail);
          t_avail += 1;
          ser.close();
          ser.open();
          ser.flush();
          msg.unavailability = t_avail;
      }
        msg.header.stamp = ros::Time::now();
        altitude.publish(msg);
        response.clear(); //delete objects within vector
        std::vector<unsigned char>().swap(response); // Frees memory
        ser.flush();
        duration.sleep();

    }
    //ser.write(airspeed_off);
}
