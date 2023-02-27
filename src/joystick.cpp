#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "joystick/c++/joy.h"
#include <sensor_msgs/Imu.h>
#include "utils/quaternion2euler.hpp"

class JoystickNode{
    /*
    * Class that launchs a ROS node that listens directly to a PS3 controller.
    * It publishes to two topics:
    *   "joy turning"   : Float64 -> [-1, 0, 1] Direction for the turning of the robot
    *   "joy direction" : Float64 -> [-pi, pi]  Direction for cartesian movement of the robot
    * 
    */
    private:
        // Variables
        double turn_dir_    = 0.0;
        double command_dir_ = 0.0;
        Joystick* joy_;
        int16_t axis_x_, axis_y_;

    public:
        explicit JoystickNode(void){};
        void update_joystick(void);
        //void get_turn_dir(void);
        //void get_command_dir(void);
        void run(int argc, char **argv);
     
};

void JoystickNode::update_joystick(void){
    //usleep(1000); This may or may not be necesary (??)
    this->joy_->Update();
    if (this->joy_->hasButtonUpdate())
    {
        ButtonId update    = this->joy_->getUpdatedButton();
        unsigned int value = this->joy_->getButtonState(update);
        
        if      (update == 11){this->turn_dir_  = (value==1)? -1.0 : 0.0;}
        else if (update == 10){this->turn_dir_  = (value==1)?  1.0 : 0.0;}
        else                  {this->turn_dir_  = 0.0;}

    }

    if (this->joy_->hasAxisUpdate())
    {
        AxisId update = this->joy_->getUpdatedAxis();
        int16_t value = this->joy_->getAxisState(update);

        if      (update == 0){this->axis_x_ = value;}
        else if (update == 1){this->axis_y_ = value;}

        this->command_dir_ = atan2(this->axis_y_, this->axis_x_);
    }
}



void JoystickNode::run(int argc, char **argv){
    ros::init(argc, argv, "joystick");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher joy_turn_dir_pub = n.advertise<std_msgs::Float64>("joy_turning", 1000);
    ros::Publisher joy_dir_pub      = n.advertise<std_msgs::Float64>("joy_direction",1000);

    // TODO: This must be part of the cfg file
    this->joy_ = new Joystick("/dev/input/js0");

    ROS_INFO("## Joystick node has been started ##");
    while (ros::ok()){

        std_msgs::Float64 turn_dir_msg;
        std_msgs::Float64 command_dir_msg;

        this->update_joystick();

        turn_dir_msg.data      = this->turn_dir_;
        command_dir_msg.data   = this->command_dir_;

        ROS_INFO("Turn Direction : [%f], Command Direction : [%f]", this->turn_dir_, this->command_dir_);
        
        joy_turn_dir_pub.publish(turn_dir_msg);
        joy_dir_pub.publish(command_dir_msg);
        ros::spinOnce();
        //loop_rate.sleep();

    }
    ROS_INFO("## Joystick node has been shut off ##");
}


int main(int argc, char **argv){
    JoystickNode joystick_node;
    joystick_node.run(argc,  argv);

    return 0;
}