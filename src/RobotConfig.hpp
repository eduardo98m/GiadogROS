

#ifndef ROBOTCONFIG_HPP
#define ROBOTCONFIG_HPP

struct RobotConfig{
    double sigma_0[4];     // {0.0, 3.142, 3.142, 0.0} [RAD]
    double base_frequency; // 12      [Hz]
    double H;              // 0.2     [m]
    double H_OFF         ; // 0.063   [m]
    double V_OFF         ; // 0.008   [m]
    double THIGH_LEN     ; // 0.11058 [m]
    double SHANK_LEN     ; // 0.1265  [m]
    double LEG_SPAN      ; // 0.2442  [m] 
    double control_dt    ; // 0.01    [s]
    //
    bool CARTESIAN_DELTA; // true
    bool ANGULAR_DELTA  ; // false
};

#endif