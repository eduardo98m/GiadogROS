#include <ros/ros.h>
#include "utils/quaternion2euler.hpp"
#include <sensor_msgs/Imu.h>


/**
 * @brief Convierte angulos de radianes en grados
 *        Al sumarle 360 grados nos aseguramos que el valor sea positivo
*/
double rad2degree(double rad) {
  return fmod(rad * 180/M_PI, 360);
}

void print_orientation(const sensor_msgs::Imu::ConstPtr& msg){
    Quaternion quaternion;
    quaternion.w = msg->orientation.w;
    quaternion.x = msg->orientation.x;
    quaternion.y = msg->orientation.y;
    quaternion.z = msg->orientation.z;
    EulerAngles rpy = ToEulerAngles(quaternion); 
    ROS_INFO("Orientation roll: [%f], pitch: [%f], yaw: [%f]", rad2degree(rpy.roll), rad2degree(rpy.pitch), rad2degree(rpy.yaw));
    //ROS_INFO("Linear acc x : [%f], y : [%f], z: [%f]" , msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
}


int main(int argc, char **argv) {
  // Inicializar nodo de ROS
  ros::init(argc, argv, "imu_test");
  ros::NodeHandle nh;

  ros::Subscriber  imu_sub = nh.subscribe("/imu/data", 500, print_orientation);   
  ROS_INFO("Testing the IMU");

  ros::spin();

  return 0;
}
