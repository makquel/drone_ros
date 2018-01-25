#include "MavStatus.h"

using namespace protomav;

MavStatus::MavStatus(){
  ros::NodeHandle nh;
  nh.param<double>("status_freq", freq, 1.0);
  voltage_sub = nh.subscribe<yocto::voltage_info>("/yocto/voltage_info", 100, &MavStatus::voltageCallback, this);
  ticksCount_ = 0;
  seqCount_ = 0;
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
 * TODO Implement Get
 */
int MavStatus::getBattery(void){
  return mux_battery_lvl*1000;
}

/*
 * TODO Implement different getSensorsHealth
 */
uint32_t MavStatus::getSensorsHealth(){
  return getSensorsPresent();
}

void MavStatus::send(){
  mavlink_message_t mmsg;
  //mavlink_sys_status_t status; //SYS_STATUS ( #1 )
  status.onboard_control_sensors_present = getSensorsPresent();
  status.onboard_control_sensors_enabled = getSensorsEnabled();
  status.onboard_control_sensors_health = getSensorsHealth();

  //TODO Informações da Bateria
  status.load = getLoad();// TODO: Get the aux voltage from blackbox

  //status.voltage_battery = 7000;//onboard_battery*100/18.5;
  status.voltage_battery = getBattery();
  status.current_battery = -1;
  status.battery_remaining = -1;

  //TODO Comunication status
  status.drop_rate_comm = 0;
  status.errors_comm = 0;

  //TODO Radio status
  radio_status.rssi = 200;

  // TODO Home status
  home.latitude = -22.8518005*1.0e7;
  home.longitude = -47.1276469*1.0e7;
  home.altitude = 0;

  mavlink_msg_home_position_encode(system_id, comp_id, &mmsg, &home);
  mavlink_msg_radio_status_encode(system_id, comp_id, &mmsg, &radio_status);
  mavlink_msg_sys_status_encode(system_id, comp_id, &mmsg, &status);
  MavMessenger::send(mmsg);
  //ROS_INFO_THROTTLE(1,"SYS_STATUS: broadcasting");
}

void MavStatus::voltageCallback(const yocto::voltage_info::ConstPtr& voltage){
  /*mav_soc.id = 1; //On board computer
  mav_soc.type = 4; //MAV_BATTERY_TYPE_NIMH
  mav_soc.battery_function = 4; //MAV_BATTERY_TYPE_PAYLOAD
  //float on_board_battery_voltage = 16;
  //mav_soc.battery_remaining = on_board_battery_voltage*100/18.5;
  mav_soc.battery_remaining = 80;*/

  //status.voltage_battery = 18000;//onboard_battery*100/18.5;
  //status.current_battery = -1;
  //status.battery_remaining = -1;
  //if(!sync) {send();}
  //ROS_INFO_THROTTLE(1,"BATT_STATUS: measuring");
  //mux_battery_lvl = voltage->current_value_4;
  //ROS_INFO("Seq %d", seqCount_);
  switch (seqCount_){
    case 0: mux_battery_lvl = voltage->current_value_1; break;
    case 1: mux_battery_lvl = voltage->current_value_2; break;
    case 2: mux_battery_lvl = voltage->current_value_3; break;
    case 3: mux_battery_lvl = voltage->current_value_4; break;
    default: break;
  }

  if (ticksCount_ > 7*2){
    ticksCount_ = 0;
    if (seqCount_ <= 3){
      seqCount_ ++;
    }else{
      seqCount_ = 0;
    }
  }
  ticksCount_ ++;
  if(!sync) {send();}
}



int main(int argc,  char** argv) {
 ros::init(argc, argv, "mav_status");

 MavStatus status;

 status.run();
 ros::spin();
 return 0;
}
