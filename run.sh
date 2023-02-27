#!/bin/bash 

source /opt/ros/noetic/setup.bash 
source /catkin_ws/devel/setup.bash 

# echo -e "Running \033[1mi2cpwm_board\033[0m node."
# roslaunch i2cpwm_board i2cpwm_node.launch &
# sleep 3.0 

echo -e "Running \033[1mimu_bno055\033[0m node."
roslaunch imu_bno055 imu.launch 

# sleep 3.0 
# echo -e "Running \033[1mservos_translate\033[0m node."
# roslaunch giadog servo_translate_node.launch
