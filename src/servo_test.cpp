#include <ros/ros.h>
#include <i2cpwm_board/ServoArray.h>
#include <i2cpwm_board/ServoConfig.h>
#include <i2cpwm_board/ServosConfig.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <iostream>

// CONSTANTES
const int N_SERVOS = 12;
const float DELTA = 0.01;
// Arreglo de centros de los servos
const int SERVO_CENTERS[N_SERVOS] = {
  361, 361, 361, 
  361, 361, 361,
  361, 361, 361,
  361, 361, 361
}; 
// Arreglo de rangos de los servos
const int SERVO_RANGES[N_SERVOS] = {
  180, 180, 180, 
  180, 180, 180,
  180, 180, 180,
  180, 180, 180
}; 
const int SERVO_DIRECTIONS[N_SERVOS] = {
  1, 1, 1, 
  1, 1, 1,
  1, 1, 1,
  1, 1, 1
};
const short ABSOLUTE = 1;

// VARIABLES GLOBALES
// Arreglo de ángulos actuales de los servos
int servo_angles_absolute[N_SERVOS] = {
  361, 361, 361, 
  361, 361, 361,
  361, 361, 361,
  361, 361, 361
}; 
float servo_angles_proportional[N_SERVOS] = {
  0, 0, 0, 
  0, 0, 0,
  0, 0, 0,
  0, 0, 0
}; 
// Índice del servo actualmente seleccionado
int servo_index = 1; 

/*
 * Realiza la configuración inicial de los Servos.
 */
void configureServos(ros::NodeHandle &nh) {
  // Crear mensaje de configuración
  i2cpwm_board::ServosConfig srv;
  
  // Configurar cada servo
  for (int i = 1; i <= N_SERVOS; i++) {
    // Definir centro, rango y dirección para cada servo
    i2cpwm_board::ServoConfig servo_config;
    servo_config.servo = i;
    servo_config.center = SERVO_CENTERS[i-1];
    servo_config.range = SERVO_RANGES[i-1];
    servo_config.direction = SERVO_DIRECTIONS[i-1];
    srv.request.servos.push_back(servo_config);
  }
  
  // Llamar al servicio para configurar los servos
  ros::ServiceClient client = nh.serviceClient<i2cpwm_board::ServosConfig>("/config_servos");
  if (client.call(srv)) {
    ROS_INFO("Servos configurados correctamente");
  } else {
    ROS_ERROR("Error al configurar los servos");
  }
}

/*
 * Publica el angulo de un servo
 */
void publishServoAbsoluteAngle(ros::Publisher &pub) {
  i2cpwm_board::ServoArray msg;
  i2cpwm_board::Servo servo_msg;

  // El índice es base 0, mientras que el número del servo es base 1
  servo_msg.servo = servo_index + 1; 
  servo_msg.value = servo_angles_absolute[servo_index];
  msg.servos.push_back(servo_msg);

  pub.publish(msg);
}

/*
 * Publica el angulo de un servo
 */
void publishServoProportionalAngle(ros::Publisher &pub) {
  i2cpwm_board::ServoArray msg;
  i2cpwm_board::Servo servo_msg;

  // El índice es base 0, mientras que el número del servo es base 1
  servo_msg.servo = servo_index + 1; 
  servo_msg.value = servo_angles_proportional[servo_index];
  msg.servos.push_back(servo_msg);

  pub.publish(msg);
}


int main(int argc, char **argv) {
  // Inicializar nodo de ROS
  ros::init(argc, argv, "servo_test");
  ros::NodeHandle nh;
  char c;

  // Configuramos el sistema para permitir obtener un solo caracter sin 
  // presionar enter
  static struct termios oldt, newt;

  /*tcgetattr gets the parameters of the current terminal
  STDIN_FILENO will tell tcgetattr that it should write the settings
  of stdin to oldt*/
  tcgetattr(STDIN_FILENO, &oldt);
  /*now the settings will be copied*/
  newt = oldt;

  /*ICANON normally takes care that one line at a time will be processed
  that means it will return if it sees a "\n" or an EOF or an EOL*/
  newt.c_lflag &= ~(ICANON); 

  /*Those new settings will be set to STDIN
  TCSANOW tells tcsetattr to change attributes immediately. */
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  // Configurar los servos
  configureServos(nh);

  // Crear publicador para los ángulos de los servos
  ros::Publisher servo_pub;
  if (ABSOLUTE) {
    servo_pub = nh.advertise<i2cpwm_board::ServoArray>("/servos_absolute", 10);
    ROS_INFO("Current servo: %d - Angle: %d", servo_index, servo_angles_absolute[servo_index]);
  }
  else {
    servo_pub = nh.advertise<i2cpwm_board::ServoArray>("/servos_proportional", 10);
    ROS_INFO("Current servo: %d - Angle: %f", servo_index, servo_angles_proportional[servo_index]);
  }

  // Bucle principal
  while (ros::ok()) {
    // Leemos la entrada de teclado
    c = getchar();

    if (c == 'k')
      break;

    switch(c) {
      case 'a':
        // Mover el índice a la izquierda
        servo_index = (servo_index + N_SERVOS - 1) % N_SERVOS;
        break;
      
      case 'd':
        // Mover el índice a la derecha
        servo_index = (servo_index + 1) % N_SERVOS;
        break;

      case 'w':
        // Incrementar el ángulo del servo actual
        servo_angles_absolute[servo_index]++;
        if (servo_angles_absolute[servo_index] > SERVO_CENTERS[servo_index] + SERVO_RANGES[servo_index]) {
          servo_angles_absolute[servo_index] = SERVO_CENTERS[servo_index] + SERVO_RANGES[servo_index];
        }
        servo_angles_proportional[servo_index] += DELTA;
        if (servo_angles_proportional[servo_index] > 1) {
          servo_angles_proportional[servo_index] = 1;
        }
        break;

      case 's':
        // Decrementar el ángulo del servo actual
        servo_angles_absolute[servo_index]--;
        if (servo_angles_absolute[servo_index] < SERVO_CENTERS[servo_index] - SERVO_RANGES[servo_index]) {
          servo_angles_absolute[servo_index] = SERVO_CENTERS[servo_index] - SERVO_RANGES[servo_index];
        }
        servo_angles_proportional[servo_index] -= DELTA;
        if (servo_angles_proportional[servo_index] < -1) {
          servo_angles_proportional[servo_index] = -1;
        }
        break;

      default: 
        break;
    }

    // Publicar ángulo del servo actual
    if (ABSOLUTE) {
      publishServoAbsoluteAngle(servo_pub);
      ROS_INFO("Current servo: %d - Angle: %d", servo_index, servo_angles_absolute[servo_index]);
    }
    else {
      publishServoProportionalAngle(servo_pub);
      ROS_INFO("Current servo: %d - Angle: %f", servo_index, servo_angles_proportional[servo_index]);
    }
  }

  return 0;
}