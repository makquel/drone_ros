#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Byte.h>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>

serial::Serial ser;

bool CTS_status;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data.c_str());
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher airspeed = nh.advertise<std_msgs::Float32>("airspeed_msg", 1000);

    unsigned char xml_parser_on[] = { 0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0x30, 0xEF, 0xE8, 0xFF, 0xFE, 0x02, 0x44, 0x31, 0x05, 0x72 };
    std::vector<unsigned char> airspeed_on(xml_parser_on, xml_parser_on+21);

    unsigned char xml_parser_off[] = { 0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0x30, 0xEF, 0xE8, 0xFF, 0xFE, 0x02, 0x44, 0x31, 0x04, 0x73 };
    std::vector<unsigned char> airspeed_off(xml_parser_off, xml_parser_off+21);

    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
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
        ser.write(airspeed_on);
    }else{
        return -1;
    }

    ros::Rate loop_rate(1); //1Hz
    ros::Duration duration(1./4.); //0.25s

    
    std::vector<unsigned char> hexstring;
    hexstring.push_back(0x7E);
    hexstring.push_back(0x00);
    hexstring.push_back(0x0F);
    hexstring.push_back(0x17);
    hexstring.push_back(0x01);
    hexstring.push_back(0x00);
    hexstring.push_back(0x7D);
    hexstring.push_back(0x33);
    hexstring.push_back(0xA2);
    hexstring.push_back(0x00);
    hexstring.push_back(0x40);
    hexstring.push_back(0x30);
    hexstring.push_back(0xEF);
    hexstring.push_back(0xE8);
    hexstring.push_back(0xFF);
    hexstring.push_back(0xFE);
    hexstring.push_back(0x02);
    hexstring.push_back(0x49);
    hexstring.push_back(0x53);
    hexstring.push_back(0x50);

    std::vector<unsigned char> response;
    duration.sleep(); //Xbee timer after power*up

    while(ros::ok()){

        ros::spinOnce();

        ser.write(hexstring);
        duration.sleep();

        //std::cout << "CTS:" << ser.getCTS() << "\n";
        //std::cout << "Buffer size: " << ser.available() << "\n";    
        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            std_msgs::Float32 result;
           
            ser.read(response, ser.available());
            //std::cout << "Vector size: " << response.size() << "\n"; 
            //for(int i=0; i<response.size(); ++i)
            //    std::cout << response[i] << ' ';
            float x = ((response[25])*256 + (response[26])*1)*3.3;
            //std::cout << "AirSpeed_ping: [" << x << "]\n";
            result.data = x;
            //result.data = response[25];
            ROS_INFO_STREAM("Airspeed[mV]: " << result.data);
            airspeed.publish(result);
        }
        response.clear(); //delete objects within vector
        std::vector<unsigned char>().swap(response); // Frees memory
        //loop_rate.sleep();

    }
    ser.write(airspeed_off);
}