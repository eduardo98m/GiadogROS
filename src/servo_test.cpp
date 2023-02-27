#include <fcntl.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include "utils/config.hpp"
#include "utils/translate.hpp"
#include <i2cpwm_board/IntValue.h>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/ServoConfig.h>
#include <i2cpwm_board/ServosConfig.h>

// VARIABLES GLOBALES
std::vector<float> angles; 
int servo_index = 0; 

/*
 * Publica el angulo de un servo
 */
void publishServoAngle(ros::Publisher &pub) {
  i2cpwm_board::ServoArray msg;
  i2cpwm_board::Servo servo_msg;

  // El índice es base 0, mientras que el número del servo es base 1
  servo_msg.servo = servo_index + 1; 
  servo_msg.value = angles[servo_index];
  msg.servos.push_back(servo_msg);

  pub.publish(msg);
}

int main(int argc, char **argv) {
  // Inicializar nodo de ROS
  ros::init(argc, argv, "servo_test");
  ros::NodeHandle nh;
  char c;

  ROS_INFO("Testing servos using %s", Config::SERVOS.TRANSLATE ? "/servos_translate" : "/servos_absolute");

  // Asignamos los angulos iniciales
  for (int i = 0; i < Config::SERVOS.N_SERVOS; i++) {
    angles.push_back(
      Config::SERVOS.TRANSLATE ? M_PI : 
        translate(i, degrees2rad(Config::SERVOS.SERVOS[i].INIT_ANGLE)));
  }

  // Configuramos solo si no usamos el translate
  if (!Config::SERVOS.TRANSLATE) {
    Config::SERVOS.configureServos(nh);
  }

  // Configuramos el sistema para permitir obtener un solo caracter sin 
  // presionar enter
  static struct termios oldt, newt;

  // tcgetattr gets the parameters of the current terminal
  // STDIN_FILENO will tell tcgetattr that it should write the settings
  // of stdin to oldt
  tcgetattr(STDIN_FILENO, &oldt);
  /*now the settings will be copied*/
  newt = oldt;

  // ICANON normally takes care that one line at a time will be processed
  // that means it will return if it sees a "\n" or an EOF or an EOL
  newt.c_lflag &= ~(ICANON); 

  // Those new settings will be set to STDIN
  // TCSANOW tells tcsetattr to change attributes immediately. 
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // Crear publicador para los ángulos de los servos
  ros::Publisher servo_pub;
  servo_pub = nh.advertise<i2cpwm_board::ServoArray>(
    Config::SERVOS.TRANSLATE ? "/servos_translate" : "/servos_absolute", 
    1000
  );
  ROS_INFO("Current servo: %d - Angle: %f", servo_index, angles[servo_index]);

  // Bucle principal
  while (ros::ok()) {
    // Leemos la entrada de teclado
    c = getchar();

    if (c == 'k')
      break;

    switch(c) {
      case 'a':
        // Mover el índice a la izquierda
        servo_index = (servo_index + Config::SERVOS.N_SERVOS - 1) % Config::SERVOS.N_SERVOS;
        break;
      
      case 'd':
        // Mover el índice a la derecha
        servo_index = (servo_index + 1) % Config::SERVOS.N_SERVOS;
        break;

      case 'w':
        // Incrementar el ángulo del servo actual
        angles[servo_index] += Config::SERVOS.TRANSLATE ? Config::SERVOS.DELTA : 1;
        publishServoAngle(servo_pub);
        break;

      case 's':
        // Decrementar el ángulo del servo actual
        angles[servo_index] -= Config::SERVOS.TRANSLATE ? Config::SERVOS.DELTA : 1;
        publishServoAngle(servo_pub);
        break;

      default: 
        break;
    }

    // Publicar ángulo del servo actual
    ROS_INFO("Current servo: %d - Angle: %f", servo_index, angles[servo_index]);
  }

  return 0;
}