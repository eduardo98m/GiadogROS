#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Eigen/Dense>

#include <sstream>

#include "kinematics/kinematicsPipeline.hpp"
#include "RobotConfig.hpp"



// We define a few global variables related that will be linked to our 



class KinematcsNode{
    
    private:
        // Variables
        double roll_  = 0.0;
        double pitch_ = 0.0;
        RobotConfig robot_config_;
        double turn_dir_ = 0.0;
        double command_dir_ = 0.0;

        std::tuple<
                Eigen::VectorXd, 
                Eigen::VectorXd, 
                Eigen::Vector4d, 
                Eigen::Vector4d, 
                Eigen::Vector4d,
                Eigen::Vector4d> kinematics_pipeline_output_;

        // This is temporal (for testing purposes)
        robot_config.sigma_0         = {0.0, 3.142, 3.142, 0.0};
        robot_config.base_frequency  = 12;
        robot_config.H               = 0.2;
        robot_config.H_OFF           = 0.069109;
        robot_config.V_OFF           = 0.00878;
        robot_config.THIGH_LEN       = 0.107734;
        robot_config.SHANK_LEN       = 0.13575;
        robot_config.LEG_SPAN        = 0.252264;
        robot_config.control_dt      = 0.01;
        robot_config.ANGULAR_DELTA   = true;
        robot_config.CARTESIAN_DELTA = true;
    
    public:
        explicit KinematcsNode(void) { };

        void orientationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void commandDirCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        void turnDirCallback(const std_msgs::Float64::ConstPtr& msg);   
        void run(int argc, char **argv);     

}

void KinematcsNode::orientationCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  this->roll_, this->pitch_, _ = msg.data;
}

void KinematcsNode::commandDirCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  this->command_dir_ = msg.data;
}

void KinematcsNode::turnDirCallback(const std_msgs::Float64::ConstPtr& msg)
{
  this->turn_dir_   = msg.data;
}


void KinematcsNode::run(int argc, char **argv){
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle n;

    // Publishers
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Publisher joint_target_angles_pub = n.advertise<std_msgs::Float64MultiArray>("joint target angles", 20);

    // Subscribers
    ros::Subscriber joy_dir_sub     = n.subscribe("joy direction", 20, this->commandDirCallback);
    ros::Subscriber joy_turn_sub    = n.subscribe("joy turning",   20, this->turnDirCallback);
    ros::Subscriber orientation_sub = n.subscribe("orientation",   20, this->orientationCallback);

    ROS_INFO("## Kinematics node has been started ##");
    while (ros::ok()){
        // Joint target angles message
        std_msgs::Float64MultiArray joint_angles_msg;
        double elapsed_time = ros::Time::now().toSec() - time_o;
        
        // Eventually the action wil be a message to which this node will be subscribed
        Eigen::VectorXd action;
        action.setZero(16);
        
        this->kinematics_pipeline_output_ = kinematicsPipeline(action,
                                                                this->turn_dir_,
                                                                this->command_dir_,
                                                                this->roll_, 
                                                                this->pitch_, 
                                                                elapsed_time,
                                                                &this->robot_config);

        Eigen::VectorXd joints_target    = std::get<0>(kinematics_pipeline_output);
        Eigen::Vector4d feet_target_pos  = std::get<1>(kinematics_pipeline_output);
        Eigen::Vector4d FTG_frequencies  = std::get<2>(kinematics_pipeline_output);
        Eigen::Vector4d FTG_sin_phases   = std::get<3>(kinematics_pipeline_output);
        Eigen::Vector4d FTG_cos_phases   = std::get<4>(kinematics_pipeline_output);
        Eigen::Vector4d FTG_phases       = std::get<5>(kinematics_pipeline_output);

        // We put the data from the joints target into the message
        joint_angles_msg.data = joints_target.data();
        
        // We reset the in initial time
        // We do this to prevent overflow
        if (elapsed_time > 5){time_o = ros::Time::now().toSec()};
        // We then proceed to publish the msgs
        //chatter_pub.publish(msg); // TODO
        joint_target_angles_pub.publish(joint_angles_msg)
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("## Kinematics node has been shutdown ##");

}


int main(int argc, char **argv){
    kinematics_node = KinematcsNode();
    kinematics_node.run(argc,  **argv);

    return 0;
}

