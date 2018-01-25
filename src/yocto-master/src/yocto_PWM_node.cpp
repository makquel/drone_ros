#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
//#include <iostream>
//#include <libusb.h>

#include "yocto/yocto_api.h"
#include "yocto/yocto_pwminput.h"
#include "yocto/voltagem_api.h"
#include "yocto/yocto_voltage.h"
#include <yocto/voltage_info.h>
#include <yocto/PWM_info.h>
/**
 * Código para a publicação de mensagens do tipo yocto/PWM_info no tópico /yocto/pwm_info
 */

using namespace std;

int clean_stdin()
{
    while (getchar()!='\n');
    return 1;
}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line. For programmatic
     * remappings you can use a different version of init() which takes remappings
     * directly, but for most command-line programs, passing argc and argv is the easiest
     * way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "yocto_node");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher pub1 = n.advertise<yocto::PWM_info>("/yocto/pwm_info", 1000); //avisa que irá publicar no tópico /yocto/pwm_info
    ros::Publisher pub2 = n.advertise<yocto::voltage_info>("/yocto/voltage_info", 1000);
    float sampling_rate = 50.0; //Hz
    ros::Duration duration(1./sampling_rate); ///1/Ts


    string       errmsg;
    string       target1 = "YPWMRX01-37171"; //
    string       target2 = "YPWMRX01-43C43"; //
    string       onboard_pack = "VOLTAGE1-73EA0"; // Onboard computer
    string       front_pack = "VOLTAGE1-73E4E"; // front power pack
    string       rear_back = "VOLTAGE1-756CE"; // rear power pack
    string       aux_pack = "VOLTAGE1-7569B"; // aux pack
    YPwmInput   *pwm;
    YPwmInput   *pwm1;
    YPwmInput   *pwm2;
    YPwmInput   *pwm3;
    YPwmInput   *pwm4;
    YVoltage    *volt;
    YVoltage    *sensorDC1;
    YVoltage    *sensorDC2;
    YVoltage    *sensorDC3;
    YVoltage    *sensorDC4;
    YModule     *m;
    bool pwm_a_available = false;
    bool pwm_b_available = false;
    bool volt1 = false;
    bool volt2 = false;
    bool volt3 = false;
    bool volt4 = false;

    YAPI::DisableExceptions();

    // Setup the API to use local USB devices
    if (YAPI::RegisterHub("usb", errmsg) != YAPI_SUCCESS) {
        cerr << "RegisterHub error: " << errmsg << endl;
        return 1;
    }

    // retreive any pwm input available
    pwm = YPwmInput::FindPwmInput(target1 + ".pwmInput1");
    if (pwm == NULL) {
      ROS_WARN("No YOCTO PWM module connected, please check module connection");
      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }

    // we need to retreive both channels from the device.
    if (pwm->isOnline()) {
        m = pwm->get_module();
        pwm1 = YPwmInput::FindPwmInput(m->get_serialNumber() + ".pwmInput1");
        pwm2 = YPwmInput::FindPwmInput(m->get_serialNumber() + ".pwmInput2");
        pwm_a_available = true;
    } else {
      ROS_WARN("No YOCTO PWM A module connected, please check module connection");
      pwm_a_available = false;

      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }


    pwm = YPwmInput::FindPwmInput(target2 + ".pwmInput1");
    if (pwm == NULL) {
      ROS_WARN("No YOCTO PWM module connected, please check module connection");
      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }

    // we need to retreive both channels from the device.
    if (pwm->isOnline()) {
        m = pwm->get_module();
        pwm3 = YPwmInput::FindPwmInput(m->get_serialNumber() + ".pwmInput1");
        pwm4 = YPwmInput::FindPwmInput(m->get_serialNumber() + ".pwmInput2");
        pwm_b_available = true;
    } else {
      ROS_WARN("No YOCTO PWM B module connected, please check module connection");
      pwm_b_available = false;
      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }

    volt = YVoltage::FindVoltage(onboard_pack + ".voltage1");
    if (volt == NULL) {
      ROS_WARN("No YOCTO VOLTAGE module connected, please check module connection");
      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }

    // we need to retreive both DC and AC voltage from the device.
    if (volt->isOnline()) {
        m = volt->get_module();
        sensorDC1 = YVoltage::FindVoltage(m->get_serialNumber() + ".voltage1");
        volt1 = true;
    } else {
      ROS_WARN("No On-Board battery sensor connected, please check module connection");
      //cerr << "No module connected (Check cable)" << endl;
      //exit(1);
    }

    // retreive any voltage sensor (can be AC or DC)
    //sensor = YVoltage::FirstVoltage();
  volt = YVoltage::FindVoltage(front_pack + ".voltage1");
  if (volt == NULL) {
    ROS_WARN("No YOCTO VOLTAGE module connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }

  // we need to retreive both DC and AC voltage from the device.
  if (volt->isOnline()) {
      m = volt->get_module();
      sensorDC2 = YVoltage::FindVoltage(m->get_serialNumber() + ".voltage1");
      volt2 = true;
  } else {
    ROS_WARN("No Front power pack sensor connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }

  // retreive any voltage sensor (can be AC or DC)
  //sensor = YVoltage::FirstVoltage();
  volt = YVoltage::FindVoltage(rear_back + ".voltage1");
  if (volt == NULL) {
    ROS_WARN("No YOCTO VOLTAGE module connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }

  // we need to retreive both DC and AC voltage from the device.
  if (volt->isOnline()) {
    m = volt->get_module();
    sensorDC3 = YVoltage::FindVoltage(m->get_serialNumber() + ".voltage1");
    volt3 = true;
  } else {
    ROS_WARN("No Rear power pack sensor connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }
  // retreive any voltage sensor (can be AC or DC)
  //sensor = YVoltage::FirstVoltage();
  volt = YVoltage::FindVoltage(aux_pack + ".voltage1");
  if (volt == NULL) {
    ROS_WARN("No YOCTO VOLTAGE module connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }

  // we need to retreive both DC and AC voltage from the device.
  if (volt->isOnline()) {
    m = volt->get_module();
    sensorDC4 = YVoltage::FindVoltage(m->get_serialNumber() + ".voltage1");
    volt4 = true;
  } else {
    ROS_WARN("No Aux battery sensor connected, please check module connection");
    //cerr << "No module connected (Check cable)" << endl;
    //exit(1);
  }




    yocto::PWM_info mtr1;     //objeto da mensagem que será publicada
    yocto::voltage_info mtr2;     //objeto da mensagem que será publicada


    while (ros::ok()){
      //mtr1.duty_cycle_1    = pwm1->get_dutyCycle();
      if (pwm_a_available == true) {
        mtr1.frequency_1     = pwm1->get_frequency();
        mtr1.duty_cycle_1    = pwm1->get_dutyCycle();
        mtr1.frequency_2     = pwm2->get_frequency();
        mtr1.duty_cycle_2    = pwm2->get_dutyCycle();
      } else{
        mtr1.frequency_1     = 0;
        mtr1.duty_cycle_1    = 0;
        mtr1.frequency_2     = 0;
        mtr1.duty_cycle_2    = 0;
      }
      if (pwm_b_available == true) {
        mtr1.frequency_3     = pwm3->get_frequency();
        mtr1.duty_cycle_3    = pwm3->get_dutyCycle();
        mtr1.frequency_4     = pwm4->get_frequency();
        mtr1.duty_cycle_4    = pwm4->get_dutyCycle();
      } else{
        mtr1.frequency_3     = 0;
        mtr1.duty_cycle_3    = 0;
        mtr1.frequency_4     = 0;
        mtr1.duty_cycle_4    = 0;
      }
      /*ROS_INFO("-----------------------------\n");
      ROS_INFO("Frequency:          %f", mtr1.frequency_1);
      ROS_INFO("Duty Cycle 1:       %f", mtr1.duty_cycle_1);
      ROS_INFO("Frequency:        %f", mtr1.frequency_2);
      ROS_INFO("Duty Cycle 2:       %f", mtr1.duty_cycle_2);
      ROS_INFO("Frequency:        %f", mtr1.frequency_3);
      ROS_INFO("Duty Cycle 3:       %f", mtr1.duty_cycle_3);
      ROS_INFO("Frequency:        %f", mtr1.frequency_4);
      ROS_INFO("Duty Cycle 4:       %f", mtr1.duty_cycle_4);*/
      if (volt1 == true){
        mtr2.current_value_1    = sensorDC1->get_currentValue();
      }
      if (volt2 == true){
        mtr2.current_value_2    = sensorDC2->get_currentValue();
      }
      if (volt3 == true){
        mtr2.current_value_3    = sensorDC3->get_currentValue();
      }
      if (volt4 == true){
        mtr2.current_value_4    = sensorDC4->get_currentValue();
      }
      /*ROS_INFO("-----------------------------\n");
      ROS_INFO("Onboard_CPU:          %f", mtr2.current_value_1);
      ROS_INFO("Front_power_pack:          %f", mtr2.current_value_2);
      ROS_INFO("Rear_power_pack:          %f", mtr2.current_value_3);
      ROS_INFO("Aux_RC:          %f", mtr2.current_value_4);*/
      //ROS_ERROR("ROS_ERROR");

      pub1.publish(mtr1);
      pub2.publish(mtr2);
      ros::spinOnce();
      duration.sleep();

    }

    return 0;
}
