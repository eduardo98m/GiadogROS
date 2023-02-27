#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include "utils/config.hpp"
#include "utils/translate.hpp"
#include <i2cpwm_board/Servo.h>
#include <i2cpwm_board/IntValue.h>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/ServoConfig.h>
#include <i2cpwm_board/ServosConfig.h>

ros::Publisher servo_pub;
ros::Time last_time;

/**
 * @brief Recibe un mensaje ServoArray con nuevas posiciones para los servos
 * normalizadas y en radianes, y las traduce a grados y en sus rangos originales.
 * 
 * @param servo Indice del servo.
 * @param rad_value Angulo en radianes al que se desea mover el servo.
 * @return Angulo en grados en el rango correcto del servo.
*/
void servos(const i2cpwm_board::ServoArray::ConstPtr& msg) {
  i2cpwm_board::ServoArray new_msg;

  for(std::vector<i2cpwm_board::Servo>::const_iterator sp = msg->servos.begin(); sp != msg->servos.end(); ++sp) {
    i2cpwm_board::Servo servo_msg;

    servo_msg.servo = sp->servo;
    servo_msg.value = translate(sp->servo - 1, sp->value);
    new_msg.servos.push_back(servo_msg);

    ROS_INFO("servo[%d] = %f", servo_msg.servo - 1, servo_msg.value);
  }

  servo_pub.publish(new_msg);

  // Calcular la diferencia de tiempo desde la última llamada a la función
  ros::Duration delta_time = ros::Time::now() - last_time;
  double frequency = 1.0 / delta_time.toSec();
  ROS_INFO("Received %d servo messages at a frequency of %.2f Hz", (int)msg->servos.size(), frequency);
  last_time = ros::Time::now();
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "servos_controller");
  ros::NodeHandle nh;

  last_time = ros::Time::now();

  // Configurar los servos
  Config::SERVOS.configureServos(nh);

  servo_pub = nh.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 1000);

  ros::Subscriber controller = nh.subscribe("servos_translate", 1000, servos);

  ros::spin();

  return 0;
}