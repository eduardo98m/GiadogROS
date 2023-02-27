#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <i2cpwm_board/IntValue.h>
#include <i2cpwm_board/ServoConfig.h>
#include <i2cpwm_board/ServosConfig.h>

namespace Config {
  /**
   * @brief Configuracion del Robot.
  */
  class RobotConfig 
  {
    public:
      const bool   CARTESIAN_DELTA; 
      const bool   ANGULAR_DELTA  ; 
      const double BASE_FREQUENCY; 
      const double H;
      const double H_OFF; 
      const double V_OFF; 
      const double THIGH_LEN; 
      const double SHANK_LEN; 
      const double LEG_SPAN; 
      const double CONTROL_DT; 
      const std::vector<double> SIGMA_0;

      RobotConfig(const YAML::Node config) :
        CARTESIAN_DELTA (config["CARTESIAN_DELTA"].as<bool>()),
        ANGULAR_DELTA   (config["ANGULAR_DELTA"].as<bool>()),
        BASE_FREQUENCY  (config["BASE_FREQUENCY"].as<int>()),
        H               (config["H"].as<double>()),
        H_OFF           (config["H_OFF"].as<double>()),
        V_OFF           (config["V_OFF"].as<double>()),
        THIGH_LEN       (config["THIGH_LEN"].as<double>()),
        SHANK_LEN       (config["SHANK_LEN"].as<double>()),
        LEG_SPAN        (config["LEG_SPAN"].as<double>()),
        CONTROL_DT      (config["CONTROL_DT"].as<double>()),
        SIGMA_0         (config["SIGMA_0"].as<std::vector<double>>())
      { }
  };

  /**
   * @brief Configuracion de un Servo 
  */
  class ServoConfig 
  {
    public:
      const int MIN_ANGLE;
      const int MAX_ANGLE;
      const int INIT_ANGLE;

      ServoConfig(const YAML::Node config) :
        MIN_ANGLE  (config["MIN_ANGLE"].as<int>()),
        MAX_ANGLE  (config["MAX_ANGLE"].as<int>()),
        INIT_ANGLE (config["INIT_ANGLE"].as<int>())
      { }
  };

  /**
   * @brief Configuracion de los Servos.
   */
  class ServosConfig
  {
    public:
      const int   N_SERVOS;
      const int   PWM_FREQ;
      const int   MS_CENTER;
      const int   MS_MIN;
      const int   MS_MAX;
      const int   DIRECTION;
      const float DELTA;
      const bool  TRANSLATE;
      const std::vector<ServoConfig> SERVOS;

      ServosConfig(const YAML::Node config) : 
        N_SERVOS  (config["N_SERVOS"].as<int>()),
        PWM_FREQ  (config["PWM_FREQ"].as<int>()),
        MS_CENTER (config["MS_CENTER"].as<int>()),
        MS_MIN    (config["MS_MIN"].as<int>()),
        MS_MAX    (config["MS_MAX"].as<int>()),
        DIRECTION (config["DIRECTION"].as<int>()),
        DELTA     (config["DELTA"].as<float>()),
        TRANSLATE (config["TRANSLATE"].as<bool>()),
        SERVOS    ([&config]() {
          std::vector<ServoConfig> servos;
          for (const auto& node : config["SERVOS"]) {
            servos.push_back(node);
          }
          return servos;
        }())
      { }

      /**
       * Calcula el centro PWM en base a la frecuencia y al PWM que representa
       * 180ª
      */
      int getPWMCenter(void) {
        return this->MS_CENTER * this->PWM_FREQ * 64 / 15625;  
      }

      /**
       * Calcula el rango PWM en base a la frecuencia y al PWM que representa
       * 180ª
      */
      int getPWMRange(void) {
        return (this->MS_MAX - this->MS_MIN) * this->PWM_FREQ * 32 / 15625;   // 2 * 4096 / 10^6
      }

      /**
       * Realiza la configuración inicial de los Servos.
      */
      void configureServos(ros::NodeHandle &nh) {
        // Crear mensaje de configuracion de la frecuencia de pwm
        i2cpwm_board::IntValue pwm_frequency;
        pwm_frequency.request.value = this->PWM_FREQ;
        ros::ServiceClient client = nh.serviceClient<i2cpwm_board::IntValue>("/set_pwm_frequency");
        if (client.call(pwm_frequency)) {
          ROS_INFO("PWM frequency configurado correctamente en %d", this->PWM_FREQ);
        } else {
          ROS_ERROR("Error al configurar el PWM frequency");
        }

        // Crear mensaje de configuración de los servos
        i2cpwm_board::ServosConfig srv;
        
        // Configurar cada servo
        for (int i = 1; i <= this->N_SERVOS; i++) {
          // Definir centro, rango y dirección para cada servo
          i2cpwm_board::ServoConfig servo_config;
          servo_config.servo     = i;
          servo_config.center    = this->getPWMCenter();
          servo_config.range     = this->getPWMRange();
          servo_config.direction = this->DIRECTION;
          ROS_INFO(
              "Configurando Servo %d | Centro: %d | Rango: %d | Direccion: %d",
              i - 1,
              servo_config.center,
              servo_config.range,
              servo_config.direction);
          srv.request.servos.push_back(servo_config);
        }
        
        // Llamar al servicio para configurar los servos
        client = nh.serviceClient<i2cpwm_board::ServosConfig>("/config_servos");
        if (client.call(srv)) {
          ROS_INFO("Servos configurados correctamente");
        } else {
          ROS_ERROR("Error al configurar los servos");
        }
      }


  };

  /**
   * @brief Clase singleton que carga la configuracion
  */
  class Config {
    public:
      /**
       * @brief Obtiene la unica instancia de la clase singleton
      */
      static Config& getInstance();

      const ServosConfig SERVOS;
      const RobotConfig  ROBOT;

    private:
      Config(const YAML::Node& config);

      static Config* instance_;
  };

  // Config singleton
  Config* Config::instance_ = nullptr;

  Config::Config(const YAML::Node &config) : 
    SERVOS (ServosConfig(config["SERVOS"])),
    ROBOT  (RobotConfig(config["ROBOT"]))
  { }

  Config& Config::getInstance() {
    if (!instance_) {
      // Cargamos el archivo de configuracion
      // Obtener la ruta del archivo fuente actual
      std::string source_file(__FILE__);
      // Eliminar el nombre del archivo fuente para obtener la ruta del directorio
      std::string source_dir = source_file.substr(0, source_file.find_last_of("\\/"));
      // Construir la ruta relativa al archivo de configuración
      std::string config_path = source_dir + "/config.yml";
      ROS_INFO_STREAM("Archivo de configuracion: \033[1;m" + config_path + "\033[0m");

      instance_ = new Config(YAML::LoadFile(config_path));
    }

    return *instance_;
  }


  Config       CONFIG = Config::getInstance();
  ServosConfig SERVOS = Config::getInstance().SERVOS;
  RobotConfig  ROBOT  = Config::getInstance().ROBOT;
}