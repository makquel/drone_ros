#include "soc2mav.h"



SoC2Mav::SoC2Mav(){
  ROS_INFO("SOC_CONSTRUCTOR");
  std::string voltage_topic;
  ros::NodeHandle nh;
  nh.getParam("voltage_topic", voltage_topic);

  nh.param("soc_mav_freq", freq, 10.0);
  sync = (freq>0.0);
  voltage_sub = nh.subscribe<yocto::voltage_info>(voltage_topic, 100, &SoC2Mav::voltageCallback, this);
}



void SoC2Mav::send(){
  mavlink_message_t mmsg;

  ros::Time now = ros::Time::now();
  /*mav_soc.id = 3;
  mav_soc.type = 1;
  mav_soc.battery_function = 3;
  mav_soc.battery_remaining = 80;*/

  //mavlink_msg_battery_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_status_t* battery_status)
  mavlink_msg_battery_status_encode( system_id,  comp_id, &mmsg,  &mav_soc);
  //mavlink_msg_sys_status_encode(system_id, comp_id, &mmsg, &status);
  MavMessenger::send(mmsg);
  //ROS_INFO_THROTTLE(1,"BATTERY_STATUS: broadcasting");
}


void SoC2Mav::voltageCallback(const yocto::voltage_info::ConstPtr& voltage){
  //ROS_INFO_THROTTLE(1,"BATTERY_STATUS: broadcasting");
  /*mav_soc.id = 1; //On board computer
  mav_soc.type = 4; //MAV_BATTERY_TYPE_NIMH
  mav_soc.battery_function = 4; //MAV_BATTERY_TYPE_PAYLOAD
  //float on_board_battery_voltage = 16;
  //mav_soc.battery_remaining = on_board_battery_voltage*100/18.5;
  mav_soc.battery_remaining = 80;*/
  mav_soc.id = 1; // 0	MAV_BATTERY_FUNCTION_UNKNOWN	Battery function is unknown
                  // 1	MAV_BATTERY_FUNCTION_ALL	Battery supports all flight systems
                  // 2	MAV_BATTERY_FUNCTION_PROPULSION	Battery for the propulsion system
                  // 3	MAV_BATTERY_FUNCTION_AVIONICS	Avionics battery
                  // 4	MAV_BATTERY_TYPE_PAYLOAD	Payload battery
  mav_soc.type = 1;  // 0	MAV_BATTERY_TYPE_UNKNOWN	Not specified.
                    // 1	MAV_BATTERY_TYPE_LIPO	Lithium polymer battery
                    // 2	MAV_BATTERY_TYPE_LIFE	Lithium-iron-phosphate battery
                    // 3	MAV_BATTERY_TYPE_LION	Lithium-ION battery
                    // 4	MAV_BATTERY_TYPE_NIMH	Nickel metal hydride battery
  mav_soc.battery_function = 3; // 0	MAV_BATTERY_FUNCTION_UNKNOWN	Battery function is unknown
                                // 1	MAV_BATTERY_FUNCTION_ALL	Battery supports all flight systems
                                // 2	MAV_BATTERY_FUNCTION_PROPULSION	Battery for the propulsion system
                                // 3	MAV_BATTERY_FUNCTION_AVIONICS	Avionics battery
                                // 4	MAV_BATTERY_TYPE_PAYLOAD	Payload battery

  //float on_board_battery_voltage = 16;
  //mav_soc.battery_remaining = on_board_battery_voltage*100/18.5;
  mav_soc.battery_remaining = 80;
  mav_soc.temperature = 25.0;
  mav_soc.voltages[0] = 3.0; //voltage->current_value_4;

  status.voltage_battery = 7.4; //voltage->current_value_4;//onboard_battery*100/18.5;
  status.current_battery = -1;
  status.battery_remaining = -1;

  if(sync){
    send();
  }
}


int main(int argc,  char** argv) {
 ros::init(argc, argv, "soc2mav");

 SoC2Mav soc2mav;
 soc2mav.run();
 ROS_DEBUG("ENTERING LOOP");
 ros::spin();
 ROS_INFO("OUT OF THE LOOP");
}
