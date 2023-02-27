#pragma once 

#include <Eigen/Dense>
#include "../utils/config.hpp"

/**
 * @brief 
 * 
 * @param ftg_freqs 
 * @param ftg_sine_phases 
 * @param command_dir 
 * @param turn_dir 
 * @param add_cartesian_delta 
 * @param add_angular_delta 
 * @param config 
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd FPH(
  Eigen::Vector4d ftg_freqs, 
  Eigen::Vector4d ftg_sine_phases, 
  double          command_dir, 
  int             turn_dir,
  bool            add_cartesian_delta,
  bool            add_angular_delta
) { 
  Eigen::MatrixXd delta = Eigen::MatrixXd::Zero(4, 3);

  for (int i = 0; i < 4; i++) {
    Eigen::Vector3d position_delta(0.0, 0.0, 0.0);
    
    // Position deltas
    
    // 0 is for the x position
    position_delta(0) = 1.7 * cos(command_dir) * Config::ROBOT.CONTROL_DT * 
      ftg_sine_phases(i) * ftg_freqs(i) * Config::ROBOT.LEG_SPAN;
    
    // 1 is for the y position
    position_delta(1) = 1.02 * sin(command_dir) * Config::ROBOT.CONTROL_DT * 
      ftg_sine_phases(i) * ftg_freqs(i) * Config::ROBOT.LEG_SPAN;

    // Rotation delta (Look Mom no branching!!)
    Eigen::Vector3d rotation_delta(0.0, 0.0, 0.0);
    
    double theta = M_PI/4;
    double phi_arc = (i == 0) * -theta + (i == 1) * -(M_PI - theta) + 
        (i == 2) *  theta + (i == 3) * (M_PI - theta);
    
    rotation_delta(0) = 0.68 * -cos(phi_arc) * Config::ROBOT.CONTROL_DT * 
      ftg_sine_phases(i) * turn_dir * ftg_freqs(i) * Config::ROBOT.LEG_SPAN;
    rotation_delta(1) = 0.68 * -sin(phi_arc) * Config::ROBOT.CONTROL_DT * 
      ftg_sine_phases(i) * turn_dir * ftg_freqs(i) * Config::ROBOT.LEG_SPAN;
    
    delta.row(i) += (
      position_delta * add_cartesian_delta + 
      rotation_delta * add_cartesian_delta
    );
    }
  return delta;
}