#include <cmath>
#include "clip.hpp"
#include "config.hpp"

/**
 * @brief Convierte angulos de radianes en grados
 *        Al sumarle 360 grados nos aseguramos que el valor sea positivo
*/
double degrees2rad(int degrees) {
  return fmod((degrees + 360) * M_PI / 180, 2 * M_PI);
}

/**
 * @brief Convierte el angulo en radianes centrado en 0 a PWM.
 * 
 * @param servo Indice del servo.
 * @param rad_value Angulo en radianes al que se desea mover el servo.
 * @return PWM que se debe enviar al servo.
*/
int translate(int servo, float rad_value) {
  // Obtenemos el angulo en radianes centrado en PI dentro de los limites del 
  // servo. Como escojimos como posicion inicial de nuestros servos 180 grados, 
  // se le suma PI radianes a el angulo (rad_value) y se le suman 2PI radianes 
  // para asegurarnos que el resultado es positivo.
  rad_value           = fmod(rad_value + 2 * M_PI, 2 * M_PI);
  float min_angle_rad = degrees2rad(Config::SERVOS.SERVOS[servo].MIN_ANGLE);
  float max_angle_rad = degrees2rad(Config::SERVOS.SERVOS[servo].MAX_ANGLE);
  float angle_rad     = clip(rad_value, max_angle_rad, min_angle_rad);

  // Convertimos los radianes a pwm
  int pwm_center = Config::SERVOS.getPWMCenter();
  int pwm_range  = Config::SERVOS.getPWMRange();
  int pwm        = angle_rad * pwm_range / M_PI +  (pwm_center - pwm_range);
  
  return pwm;
}