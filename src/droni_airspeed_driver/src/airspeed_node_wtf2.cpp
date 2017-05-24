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

#include <droni_airspeed_driver/sensor_data.h>

serial::Serial ser;
bool CTS_status;
std::string serial_port;
int serial_speed;
int sampling_rate;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data.c_str());
    ser.write(msg->data);
}

struct HexCharStruct
{
  unsigned char c;
  HexCharStruct(unsigned char _c) : c(_c) { }
};

inline std::ostream& operator<<(std::ostream& o, const HexCharStruct& hs)
{
  return (o << std::hex << (int)hs.c);
}

inline HexCharStruct hex(unsigned char _c)
{
  return HexCharStruct(_c);
}


int main (int argc, char** argv){
    ros::init(argc, argv, "droni_airspeed_node");
    ros::NodeHandle nh;
    float conv_factor = 500/pow(2,15);
    int rho = 1.225;// kg/m^3

    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher airspeed = nh.advertise<droni_airspeed_driver::sensor_data>("airspeed_msg", 5);

    //setting default device path for the sensor
    nh.param("serial_port", serial_port, std::string("/dev/ttyUSB0")); //TODO: Assing a name to specific device
    //nh.param("serial_port", serial_port, std::string("/dev/sensors/d_airspeed")); //TODO: Assing a name to specific device
    //setting default device communication speed for the sensor
    nh.param("serial_speed", serial_speed, int(115200));
    // setting default sampling rate
    nh.param("sampling_rate", sampling_rate, int(20));

    unsigned char xml_parser_on[] = { 0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0x30, 0xEF, 0xE8, 0xFF, 0xFE, 0x02, 0x44, 0x31, 0x05, 0x72 };
    std::vector<unsigned char> airspeed_on(xml_parser_on, xml_parser_on+21);

    unsigned char xml_parser_off[] = { 0x7E, 0x00, 0x10, 0x17, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0x30, 0xEF, 0xE8, 0xFF, 0xFE, 0x02, 0x44, 0x31, 0x04, 0x73 };
    std::vector<unsigned char> airspeed_off(xml_parser_off, xml_parser_off+21);

    unsigned char xml_parser_request[] = { 0x7E, 0x00, 0x10, 0x10, 0x01, 0x00, 0x7D, 0x33, 0xA2, 0x00, 0x40, 0xF9, 0x8F, 0x55, 0xFF, 0xFE, 0x00, 0x00, 0x52, 0x44, 0x89 };
    std::vector<unsigned char> airspeed_request(xml_parser_request, xml_parser_request+21);

    try
    {
        ser.setPort(serial_port);
        ser.setBaudrate(serial_speed);
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
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
        ser.write(airspeed_request);
    }else{
        return -1;
    }


    ros::Duration duration(1./float(sampling_rate)); //1/20Hz


    std::vector<unsigned char> response;
    duration.sleep(); //Xbee timer after power*up
    int16_t t_occ = 0;//occurrence
    while(ros::ok()){
      int16_t x = 0;

      ros::spinOnce();

        //ser.write(hexstring);
        ser.write(airspeed_request);
        //ROS_INFO_STREAM("Requesting measurement");

        //std::cout << "CTS:" << ser.getCTS() << "\n";
        //std::cout << "Buffer size: " << ser.available() << "\n";
        if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            std_msgs::Float32 result;
            droni_airspeed_driver::sensor_data msg;
            ser.read(response, ser.available());
            std::cout << "Vector size: " << int(response.size()) << "\n";
            if (int(response.size()) == 30){ //Size of Transmit Status + Receive Packet
              ROS_INFO_STREAM("Got a proper package");
              //for(int i=0; i<response.size(); ++i)
              //    std::cout << hex(response[i]) << std::endl;
              if(int((response[8])) == 0){
                //--ROS_INFO_STREAM("ok");
                x = ((response[27])*256 + (response[28])*1);
                //if(){}
                //std::cout << "AirSpeed_ping: [" << x << "]\n";
              }

              result.data = x;
              /*result.data *= conv_factor; //Pa
              result.data = sqrt(2*abs(result.data)/rho);
              if(x < 0){
                msg.airspeed = -1*result.data;
              }else{
                msg.airspeed = result.data;
              }*/
              ROS_INFO_STREAM("Airspeed[bitwise]: " << result.data );
              msg.header.stamp = ros::Time::now();
              airspeed.publish(msg);
            }else{
              //TODO: Implementar um filtro preditivo caso não haja uma medição
              t_occ += 1;
              //WARNING : Gambiarra!!
              ser.close();
              ser.open();
              if(ser.isOpen()){
                ROS_INFO_STREAM("Serial port soft reset:" << t_occ);
              }
            }
            ser.flush();
          }

        /*else{
          ROS_INFO_STREAM("Serial port unavailable");
          ser.close();
          ser.open();

        }*/
        response.clear(); //delete objects within vector
        std::vector<unsigned char>().swap(response); // Frees memory
        ser.flush();
        duration.sleep();
    }
    //ser.write(airspeed_off);
    //ser.close();
}
