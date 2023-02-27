#pragma once

#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include "../utils/config.hpp"

/**
 * @brief 
 * 
 * @param sigma_i_0 Leg base phase
 * @param t Elapsed time [s]
 * @param f_i Frequency of the i-th leg
 * @return std::pair<double, double> 
 */
std::pair<double, double> FTG(
  double sigma_i_0,
  double t,
  double f_i
) { 
  double H = 0.2 ;
  double position_z = 0.0;
  double sigma_i, k, h;

  sigma_i = std::fmod(sigma_i_0 + t * (f_i), (2 * M_PI));
  k       = 2 * (sigma_i - M_PI) / M_PI;
  h       = 0.6 * H;

  bool condition_1 = (k <= 1 && k >= 0);
  bool condition_2 = (k >= 1 && k <= 2);

  position_z += h * (-2 * k * k * k + 3 * k * k)  * condition_1;
  position_z += h * (2 * k * k * k - 9 * k * k + 12 * k - 4) * condition_2;

  return {position_z, sigma_i}; 
}

/**
 * @brief Compute the foot trajectories for the given frequencies, for all
 * the four legs for the current time t.
 * 
 * @param t Current time.
 * @param frequencies Vector of the four frequencies offsets of the four legs.
 * @return std::tuple<Eigen::Vector4d, Eigen::Vector4d, Eigen::Vector4d, Eigen::Vector4d> 
 */
std::tuple<Eigen::Vector4d, Eigen::Vector4d, Eigen::Vector4d, Eigen::Vector4d, Eigen::Vector4d> 
compute_foot_trajectories(double t, Eigen::Vector4d frequencies) { 
    // The foot phases are set to a trot gait.
    //double* sigma_0[4] = config->sigma_0;
    Eigen::Vector4d FTG_frequencies       = Eigen::Vector4d::Zero();
    Eigen::Vector4d FTG_sin_phases        = Eigen::Vector4d::Zero();  
    Eigen::Vector4d FTG_cos_phases        = Eigen::Vector4d::Zero();
    Eigen::Vector4d FTG_phases            = Eigen::Vector4d::Zero();
    Eigen::Vector4d target_foot_positions = Eigen::Vector4d::Zero();

    for (int i = 0; i < 4; i++) 
    {
        double f_i = frequencies[i] + Config::ROBOT.BASE_FREQUENCY;
        std::pair<double, double>  
        ftg_result               = FTG(Config::ROBOT.SIGMA_0[i], t, f_i);
        target_foot_positions(i) = ftg_result.first;
        double sigma_i           = ftg_result.second;
        FTG_frequencies[i]       = f_i;
        FTG_sin_phases[i]        = std::sin(sigma_i);
        FTG_cos_phases[i]        = std::cos(sigma_i);
        FTG_phases[i]            = sigma_i;
    }

    return {
        target_foot_positions, 
        FTG_frequencies, 
        FTG_sin_phases, 
        FTG_cos_phases,
        FTG_phases
    };
}