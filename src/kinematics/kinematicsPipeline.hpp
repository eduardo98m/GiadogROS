/*
    Authors: Amin Arriaga, Eduardo Lopez
    Project: Graduation Thesis: GIAdog

    Function that performs the kinematic pipeline for our robot (Giadog).
    
*/

#define _USE_MATH_DEFINES
#include <cmath>
#include "FPH.hpp"
#include "Gait.hpp"
#include "InvKinematics.hpp"
#include "../utils/config.hpp"


/**
 * @brief Function used to apply the control pipeline to the robot.
 *        The control pipeline is composed of the following steps:
 *         1. Calculate the FTG state for each foot.
 *         2. Sum the residuals to the FTG output foot positions
 *         3. Calculate the heuristic deltas for each foot according to the 
 *            desired turn direction and command direction. (Only if desired)
 *         4. Transform the foot positions to the hip frames
 *         5. Apply the inverse kinematics to the obtained foot positions.
 * 
 * @param action     The action to apply to the robot. The action is a vector of size 16. The last 4 positions are the frecuencies offsets of each legs
 * @param turn_dir   
 * @param command_dir 
 * @param roll 
 * @param pitch 
 * @param time 
 * @return std::tuple<Eigen::VectorXd,Eigen::VectorXd> 
 */
std::tuple<
  Eigen::VectorXd, 
  Eigen::VectorXd, 
  Eigen::Vector4d, 
  Eigen::Vector4d, 
  Eigen::Vector4d,
  Eigen::Vector4d
> kinematicsPipeline(
  Eigen::VectorXd action,
  double turn_dir,
  double command_dir,
  double roll, 
  double pitch, 
  double time
){
  // PARAMS
  bool CARTESIAN_DELTA = Config::ROBOT.CARTESIAN_DELTA;
  bool ANGULAR_DELTA   = Config::ROBOT.ANGULAR_DELTA;
  double H_OFF         = Config::ROBOT.H_OFF; // Hip offset
  double LEG_SPAN      = Config::ROBOT.LEG_SPAN; // Leg span

  //
  Eigen::Vector4d frequencies   = action.tail(4);
  Eigen::VectorXd xyz_residuals = action.head(12);
    
  // Calculate the FTG target position
  std::tuple<
    Eigen::Vector4d, 
    Eigen::Vector4d, 
    Eigen::Vector4d, 
    Eigen::Vector4d,
    Eigen::Vector4d
  > FTG_data = compute_foot_trajectories(time, frequencies);
  
  Eigen::Vector4d z_ftg           = std::get<0>(FTG_data);
  Eigen::Vector4d FTG_frequencies = std::get<1>(FTG_data);
  Eigen::Vector4d FTG_sin_phases  = std::get<2>(FTG_data);
  Eigen::Vector4d FTG_cos_phases  = std::get<3>(FTG_data);
  Eigen::Vector4d FTG_phases      = std::get<4>(FTG_data);
  Eigen::MatrixXd dir_delta = Eigen::MatrixXd::Zero(4, 3);
  
  dir_delta = FPH(
    FTG_frequencies, 
    FTG_cos_phases, 
    command_dir, 
    turn_dir,
    CARTESIAN_DELTA,
    ANGULAR_DELTA
  );

  Eigen::VectorXd feet_target_positions;
  feet_target_positions.setZero(12);
  Eigen::VectorXd joint_angles;
  joint_angles.setZero(12);

  for (int i = 0; i < 4; i++) 
  {
    Eigen::VectorXd foot_delta = dir_delta.row(i)*0.5;

    double x = foot_delta(0)             + xyz_residuals(i * 3);
    double y = foot_delta(1)             + xyz_residuals(i * 3 + 1);
    double z = z_ftg(i) + foot_delta(2)  + xyz_residuals(i * 3 + 2);
    
    feet_target_positions(i * 3) = x;
    feet_target_positions(i * 3 + 1) = y;
    feet_target_positions(i * 3 + 2) = z;

    // Transform the feet target position to the base horizontal frame
    Eigen::Vector3d r = {
        x * std::cos(pitch)  + y*std::sin(pitch)*std::sin(roll)  + z*std::sin(pitch)*std::cos(roll)  + 0,
        0                    + y*std::cos(roll)                  - z*std::sin(roll)                  + H_OFF * pow(-1, i),
        -x * std::sin(pitch) + y*std::cos(pitch)*std::sin(roll)  + z*std::cos(pitch)*std::cos(roll)  - LEG_SPAN * (1 - 0.225)
    };

    bool right_leg = i == 1 || i == 3;
    // Asing the joint angles to the joint angle vector.
    auto leg_joint_angles = solve_leg_IK(right_leg, r);
    joint_angles(i*3)   = leg_joint_angles[0];
    joint_angles(i*3+1) = leg_joint_angles[1];
    joint_angles(i*3+2) = leg_joint_angles[2];
  }
      
  return std::make_tuple(joint_angles, feet_target_positions, FTG_frequencies, FTG_sin_phases, FTG_cos_phases, FTG_phases);
}
