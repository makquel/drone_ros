#include "MavStatus.h"

using namespace protomav;

MavStatus::MavStatus(){
  ros::NodeHandle nh;
  nh.param<double>("status_freq", freq, 1.0);
}

uint32_t  MavStatus::getSensorsPresent(){
    uint32_t mask=(uint32_t)0;
    ros::NodeHandle nh;
    std::vector<std::string> v;
    bool sensor_present;
    nh.getParam("sensors", v);
    
    for(int i=0; i <v.size(); i++){
      for(int j=0; j<sensors_list.size(); j++){
	if(v[i]==sensors_list[j]) mask |= sensors_mask[i];
      }     
    }
    return mask;
}

uint32_t MavStatus::getSensorsEnabled(){
    ros::NodeHandle nh;
    uint32_t  mask = (uint32_t) 0;
    bool sensor_enabled;
    
    nh.getParam("orientation_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_YAW_POSITION;
    
    nh.getParam("gps_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_GPS;
    
    nh.getParam("acceleration_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    
    nh.getParam("magnetic_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_3D_MAG;
    
    nh.getParam("gyroscope_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    
    nh.getParam("laser_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_LASER_POSITION;
    
    nh.getParam("pressure_enabled", sensor_enabled);
    if(sensor_enabled) mask |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
  
    return mask;
}

/*
 * TODO Implement GetLoad - CPU LOAD
 */
int MavStatus::getLoad(){
  return 500;
}

/*
 * TODO Implement different getSensorsHealth
 */
uint32_t MavStatus::getSensorsHealth(){
  return getSensorsPresent();  
}

void MavStatus::send(){
  mavlink_message_t mmsg;
  mavlink_sys_status_t status;
  status.onboard_control_sensors_present = getSensorsPresent();
  status.onboard_control_sensors_enabled = getSensorsEnabled();
  status.onboard_control_sensors_health = getSensorsHealth();
  
  //TODO Informações da Bateria
  status.load = getLoad();//TODO
  status.voltage_battery=500;
  status.current_battery=-1;
  status.battery_remaining=-1;
  
  //TODO Comunication status
  status.drop_rate_comm = 0;
  status.errors_comm = 0;
  
  mavlink_msg_sys_status_encode(system_id, comp_id, &mmsg, &status);
  MavMessenger::send(mmsg);
}



int main(int argc,  char** argv) {
 ros::init(argc, argv, "mav_status");

 MavStatus status;
  
 status.run();
 
 
}

