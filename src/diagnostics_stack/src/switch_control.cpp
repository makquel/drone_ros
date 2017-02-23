#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

#include <yocto/PWM_info.h>
#include <ros_pololu_servo/DigitalState.h>
#include <ros_pololu_servo/DigitalCommand.h>
#include <ros_pololu_servo/MotorCommand.h>

# define M_PI           3.14159265358979323846

float RX_received[4];
bool switch_actuator[2]; // [garateia, valvula_helio]
bool commutator; //Comutador

void  digital_received_callback(const ros_pololu_servo::DigitalState::ConstPtr& msg); //le as chaves digitais de controle (rádio controle) via pololu_maestro
void  RX_received_callback(const yocto::PWM_info::ConstPtr& msg); //le os sinais de controle dos atuadores (rádio controle) via yocpuce pwm
float servo_motor_mapping (float duty_cycle);//função de alocacao de controle
float prop_motor_mapping (float duty_cycle);//função de alocacao de controle

int main(int argc, char **argv)
{

    ros::init(argc, argv, "diagnostics_node");

    ros::NodeHandle nh;
    ros::Publisher pub_motor = nh.advertise<ros_pololu_servo::MotorCommand>("/pololu/command_motor", 1000); //avisa que vá publicar no tópico /pololu/command_motor
    ros::Subscriber sub_digital = nh.subscribe("/pololu/digital_state", 100, digital_received_callback);
    ros::Subscriber sub_yocto = nh.subscribe("/yocto/pwm_info", 100, RX_received_callback);

    float sampling_rate = 64.0; //Hz
    ros::Duration duration(1./sampling_rate); ///1/Ts


    ros_pololu_servo::MotorCommand mtr;     //objeto da mensagem que será publicada
    ROS_INFO_THROTTLE(1,"Running diagnostic's node");
    while (ros::ok()){
      if(commutator == true){
        ROS_INFO_THROTTLE(1,"autopilot mode : ON");
        mtr.speed = 1.0;
        mtr.acceleration = 1.0;

        mtr.joint_name = "prop_one";
        mtr.position =  prop_motor_mapping(RX_received[0]); //-int(89)*M_PI/180;
        pub_motor.publish(mtr);

        mtr.joint_name = "prop_two";
        mtr.position =  prop_motor_mapping(RX_received[0]);
        pub_motor.publish(mtr);
        mtr.joint_name = "prop_three";
        mtr.position =  prop_motor_mapping(RX_received[0]);
        pub_motor.publish(mtr);
        mtr.joint_name = "prop_four";
        mtr.position =  prop_motor_mapping(RX_received[0]);
        pub_motor.publish(mtr);

        mtr.joint_name = "vet_one";
        mtr.position =  servo_motor_mapping(RX_received[1]);
        pub_motor.publish(mtr);
        mtr.joint_name = "vet_two";
        mtr.position =  servo_motor_mapping(RX_received[1]);
        pub_motor.publish(mtr);
        mtr.joint_name = "vet_three";
        mtr.position =  servo_motor_mapping(RX_received[1]);
        pub_motor.publish(mtr);
        mtr.joint_name = "vet_four";
        mtr.position =  servo_motor_mapping(RX_received[1]);
        pub_motor.publish(mtr);

        /*mtr.joint_name = "leme_one";
        mtr.position =  0;//servo_motor_mapping(RX_received[2]);
        pub_motor.publish(mtr);
        mtr.joint_name = "leme_two";
        mtr.position =  0; //servo_motor_mapping(RX_received[2]);
        pub_motor.publish(mtr);
        mtr.joint_name = "leme_three";
        mtr.position =  0; //servo_motor_mapping(RX_received[3]);
        pub_motor.publish(mtr);
        mtr.joint_name = "leme_four";
        mtr.position =  0; //servo_motor_mapping(RX_received[3]);
        pub_motor.publish(mtr);*/

        //ROS_INFO_THROTTLE(1, "Position: %f", motor_mapping(RX_received[0]));

        //ROS_INFO_THROTTLE(1, "Position: %d", dummy);
        pub_motor.publish(mtr);
      }
      if (commutator == false){
        ROS_INFO_THROTTLE(1,"autopilot mode : OFF");
      }

      ros::spinOnce();
      duration.sleep();

    }

    return 0;
}

void  digital_received_callback(const ros_pololu_servo::DigitalState::ConstPtr& msg)
{
    commutator = msg->comutador;
    switch_actuator[1] = msg->val_helio;
    switch_actuator[0] = msg->garateia;
    /*int dummy;
    if ( chaves[0] == false )
        dummy = 0;
    else dummy = 1;
    ROS_INFO_THROTTLE(1, "commutator: %d", dummy);*/
}

void  RX_received_callback(const yocto::PWM_info::ConstPtr& msg)
{
    RX_received[0] = msg->duty_cycle_1; // props
    RX_received[1] = msg->duty_cycle_2; // vets
    RX_received[2] = msg->duty_cycle_3; // rudders
    RX_received[3] = msg->duty_cycle_4; // ailerons
    //ROS_INFO_THROTTLE(1,"Duty Cycle 1: %f", RX_received[0]);
}

float prop_motor_mapping (float duty_cycle)
{
    //float m_pi = 3.141592;
    float rad = (M_PI*duty_cycle/5) - 3*M_PI/2;
    rad /= 2;
    if(rad > 0){
      rad = -1*(rad + M_PI/4);
    }else if (rad <= 0){
      rad += M_PI/4;
      rad *= -1;
    }
    //rad = -M_PI/2+
    //ROS_INFO_THROTTLE(1, "rad: %f", rad);
    return rad;
}
float servo_motor_mapping (float duty_cycle)
{
  //float m_pi = 3.141592;
  float rad = (M_PI*duty_cycle/5) - 3*M_PI/2;
  rad /= 2;
  //ROS_INFO_THROTTLE(1, "rad: %f", rad);
  return rad;
}
