/*
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    Inverse kinematics class for the giadog robot. (Spot mini mini // Open Quadruped)
    
    References:
    -----------
        * Muhammed Arif Sen, Veli Bakircioglu, Mete Kalyoncu. (Sep, 2017). 
        Inverse Kinematic Analysis Of A Quadruped Robot  
        https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot

        * Some of the code was taken from the sopt_mini_mini implementation 
        of the same paper.
        https://github.com/OpenQuadruped/spot_mini_mini/blob/spot/spotmicro/Kinematics/LegKinematics.py

*/
#pragma once

#define _USE_MATH_DEFINES

#include <cmath>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include "../RobotConfig.hpp"


/**
 * @brief Calculates the leg's Inverse kinematicks parameters:
 * The leg Domain 'D' (caps it in case of a breach) and the leg's radius.
 * 
 * @param x hip-to-foot distance in x-axis
 * @param y hip-to-foot distance in y-axis
 * @param z hip-to-foot distance in z-axis
 * @param config 
 * @return std::pair<double, double> leg's Domain D and leg's outer radius
 */
std::pair<double, double> get_IK_params(
    double x, 
    double y, 
    double z, 
    RobotConfig *config
) {   
    double H_OFF      = config->H_OFF;
    double V_OFF      = config->V_OFF;
    double THIGH_LEN  = config->THIGH_LEN;
    double SHANK_LEN  = config->SHANK_LEN;

    double r_o, D, sqrt_component;

    sqrt_component = std::max(
        (double) 0.0, 
        std::pow(z, 2) + std::pow(y, 2) - std::pow(H_OFF, 2)
    );
    r_o = std::sqrt(sqrt_component) - V_OFF;
    D = std::clamp(
        (std::pow(r_o, 2) + std::pow(x, 2) - std::pow(SHANK_LEN, 2) - 
            std::pow(THIGH_LEN, 2)) / (2 * SHANK_LEN * THIGH_LEN),
        -1.0, 
        1.0
    );

    return {D, r_o};
}

/**
 * @brief Right Leg Inverse Kinematics Solver
 * 
 * @param x hip-to-foot distance in x-axis
 * @param y hip-to-foot distance in y-axis
 * @param z hip-to-foot distance in z-axis
 * @param D Leg domain
 * @param r_o Radius of the leg
 * @param config 
 * @return Eigen::Vector3d Joint Angles required for desired position. 
 *  The order is: Hip, Thigh, Shank
 *  Or: (shoulder, elbow, wrist)
 */
Eigen::Vector3d right_leg_IK(
    double x, 
    double y, 
    double z, 
    double D, 
    double r_o,
    EnvConfig *config
) { 
    double SHANK_LEN  = config->SHANK_LEN;
    double H_OFF      = config->H_OFF;

    double wrist_angle, shoulder_angle, elbow_angle;
    double second_sqrt_component, q_o;

    wrist_angle    = std::atan2(-std::sqrt(1 - std::pow(D, 2)), D);
    shoulder_angle = - std::atan2(z, y) - std::atan2(r_o, - H_OFF);
    second_sqrt_component = std::max(
        (double) 0.0,
        std::pow(r_o, 2) + std::pow(x, 2) - 
            std::pow((SHANK_LEN * std::sin(wrist_angle)), 2)
    );
    q_o = std::sqrt(second_sqrt_component);
    elbow_angle = std::atan2(-x, r_o);
    elbow_angle -= std::atan2(SHANK_LEN * std::sin(wrist_angle), q_o);

    Eigen::Vector3d joint_angles(-shoulder_angle, elbow_angle, wrist_angle);
    return joint_angles;
}

/**
 * @brief Left Leg Inverse Kinematics Solver
 * 
 * @param x hip-to-foot distance in x-axis
 * @param y hip-to-foot distance in y-axis
 * @param z hip-to-foot distance in z-axis
 * @param D Leg domain
 * @param r_o Radius of the leg
 * @param config 
 * @return Eigen::Vector3d Joint Angles required for desired position. 
 *  The order is: Hip, Thigh, Shank
 *  Or: (shoulder, elbow, wrist)
 */
Eigen::Vector3d left_leg_IK(
    double x, 
    double y, 
    double z, 
    double D, 
    double r_o,
    EnvConfig *config
) { 
    double SHANK_LEN  = config->SHANK_LEN;
    double H_OFF      = config->H_OFF;

    // Declare the variables
    double wrist_angle, shoulder_angle, elbow_angle;
    double second_sqrt_component, q_o;

    wrist_angle    = std::atan2(-std::sqrt(1 - std::pow(D, 2)), D);
    shoulder_angle = - std::atan2(z, y) - std::atan2(r_o, H_OFF);
    second_sqrt_component = std::max(
        (double) 0.0,
        std::pow(r_o, 2) + std::pow(x, 2) - 
            std::pow((SHANK_LEN * std::sin(wrist_angle)), 2)
    );
    q_o = std::sqrt(second_sqrt_component);
    elbow_angle = std::atan2(-x, r_o);
    elbow_angle -= std::atan2(SHANK_LEN * std::sin(wrist_angle), q_o);

    Eigen::Vector3d joint_angles(-shoulder_angle, elbow_angle, wrist_angle);
    // print the joint angles
    return joint_angles;
}

/**
 * @brief Calculates the leg's inverse kinematics.
 * (joint angles from xyz coordinates).
 * 
 * @param right_leg If true, the right leg is solved, otherwise the left leg 
 * is solved.
 * @param r Objective foot position in the H_i frame. (x,y,z) hip-to-foot 
 * distances in each dimension
 * @param config 
 * @return Eigen::Vector3d Leg joint angles to reach the objective foot 
 * position r. In the order:(Hip, Shoulder, Wrist). The joint angles are 
 * expresed in radians.
 */
Eigen::Vector3d solve_leg_IK(bool right_leg, Eigen::Vector3d r, EnvConfig *config)
{
    std::pair<double, double> params = get_IK_params(r(0), r(1), r(2), config);

    double D   = params.first;
    double r_o = params.second;

    return right_leg ? 
        right_leg_IK(r(0), r(1), r(2), D, r_o, config) : 
        left_leg_IK(r(0), r(1), r(2), D, r_o, config);
}
