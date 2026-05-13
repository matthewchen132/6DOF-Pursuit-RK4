#pragma once
#include <Eigen/Dense>
#include<stdio.h>

inline void display_rpy(const Eigen::MatrixXd& rpy_data) {
    char display_rpy_data;
    std::cout << "Display Aero data and coeffs? [y/n]: ";
    std::cin >> display_rpy_data;
    if(display_rpy_data == 'y' || display_rpy_data == 'Y'){
        std::cout << rpy_data << std::endl;
        std::cout << "        t        Roll     Pitch     Yaw        Roll     Pitch     Yaw " << std::endl;

    }
}

inline void display_positions(const Eigen::MatrixXd& pos_log) {
    char display_pos;
    std::cout << "Display Aero data and coeffs? [y/n]: ";
    std::cin >> display_pos;
    if(display_pos == 'y' || display_pos == 'Y'){
        std::cout << pos_log << std::endl;
        std::cout << "        t        missile_x   " << "  missile_y   " << "  missile_z   " << "target_x   " << "   target_y   " << "   target_z   " << "    a_x   " << "       a_y   " << "       a_z   "  << "     ZEM_x   " << "     ZEM_y   " << "     ZEM_z   " << " ZEM_dist   " << std::endl;
    }
}

inline void display_aero(const Eigen::MatrixXd& aero_data) {
    char display_aero_data;
    std::cout << "Display Aerodynamic Angles? [y/n]:" << std::endl;
    std::cin >> display_aero_data;
    if(display_aero_data == 'y' || display_aero_data == 'Y'){
        std::cout << aero_data << std::endl;
        std::cout << "        t        alpha     beta        alpha     beta" << std::endl;
    }
}
