#include "ros/ros.h"
#include <sstream>
#include <Eigen/Dense>
#include "utils/config.hpp"
#include <std_msgs/Float64.h>
#include <i2cpwm_board/ServoArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "kinematics/kinematicsPipeline.hpp"
#include <sensor_msgs/Imu.h>
#include "utils/quaternion2euler.hpp"
// Amin esto es c++ y lo vamos a hacer con clases.


// Variables globales
double roll  = 0.0;
double pitch = 0.0;
double turn_dir    = 0.0;
double command_dir = 0.0;
std::tuple<
  Eigen::VectorXd, 
  Eigen::VectorXd, 
  Eigen::Vector4d, 
  Eigen::Vector4d, 
  Eigen::Vector4d,
  Eigen::Vector4d
> kinematics_pipeline_output;


void commandDirCallback(const std_msgs::Float64::ConstPtr& msg)
{
  command_dir = msg->data;
}

void turnDirCallback(const std_msgs::Float64::ConstPtr& msg)
{
  turn_dir = msg->data;
}


void rpyCallback(const sensor_msgs::Imu::ConstPtr& msg){
    
    Quaternion quaternion;
    quaternion.w = msg->orientation.w;
    quaternion.x = msg->orientation.x;
    quaternion.y = msg->orientation.y;
    quaternion.z = msg->orientation.z;
    EulerAngles rpy = ToEulerAngles(quaternion); 
    //ROS_INFO("Orientation roll: [%f], pitch: [%f], yaw: [%f]", rpy.roll, rpy.pitch, rpy.yaw);
    //ROS_INFO("Linear acc x : [%f], y : [%f], z: [%f]" , msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    
    // ###############################################################################
    // We invert the roll and pitch because of the positioning of our IMU on the robot
    // ###############################################################################
    roll  = rpy.pitch;
    pitch = rpy.roll;
}




i2cpwm_board::ServoArray createServosMsg(Eigen::VectorXd angles) {
  i2cpwm_board::ServoArray msg;

  for (int i = 0; i < 1; i++)
  {
    i2cpwm_board::Servo servo_msg;

    servo_msg.servo = i + 1;
    servo_msg.value = angles[i];
    // ROS_INFO("Publishing:   servo[%d] = %f", i + 1, angles[i]*180/M_PI);
    msg.servos.push_back(servo_msg);
  }

  return msg;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "kinematics");
  ros::NodeHandle nh;

  // Publishers
  ros::Publisher servos_pub = nh.advertise<i2cpwm_board::ServoArray>("/servos_translate", 1000);

  // Subscribers
  ros::Subscriber joy_dir_sub     = nh.subscribe("/command_direction", 1000, commandDirCallback);
  ros::Subscriber joy_turn_sub    = nh.subscribe("/command_turning"  , 1000, turnDirCallback);
  ros::Subscriber imu_sub         = nh.subscribe("/imu/data"         , 1000, rpyCallback);

  // Definir la frecuencia de ejecución del bucle en Hz
  ros::Rate rate(200);

  ROS_INFO("\033[1;36m## Kinematics node has been started ##\033[0m");
  double time_o = ros::Time::now().toSec();
  double elapsed_time = 0;
  double dt = ros::Time::now().toSec();
  while (ros::ok()) {
    elapsed_time = 0.0; // ros::Time::now().toSec() - time_o;
    ROS_INFO("DT: %f", ros::Time::now().toSec() - dt);
    dt = ros::Time::now().toSec();

    // Eventually the action wil be a message to which this node will be subscribed
    Eigen::VectorXd action;
    action.setZero(16);
    
    kinematics_pipeline_output = kinematicsPipeline(
      action,
      turn_dir,
      command_dir,
      roll, 
      pitch, 
      elapsed_time
    );

    Eigen::VectorXd joints_target = std::get<0>(kinematics_pipeline_output);

    joints_target[4]  = -joints_target[4] ;
    joints_target[5]  = -joints_target[5] ;
    joints_target[10] = -joints_target[10];
    joints_target[11] = -joints_target[11];

    joints_target.array() += M_PI;
    // We put the data from the joints target into the message
    i2cpwm_board::ServoArray angles_msg = createServosMsg(joints_target);
    
    // We reset the in initial time
    // We do this to prevent overflow
    if (elapsed_time > 5){
      time_o = ros::Time::now().toSec();
    };
    // We then proceed to publish the msgs
    servos_pub.publish(angles_msg);
    ros::spinOnce();

    // Dormir el bucle hasta alcanzar la frecuencia deseada
    rate.sleep();
  }
  ROS_INFO("\033[1;36m## Kinematics node has been shutdown ##\033[0m");

  return 0;
}
