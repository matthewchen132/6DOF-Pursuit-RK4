#pragma once
#include <Eigen/Dense>
#include <stdio.h>
#include <filesystem>
#include "Logger.hpp"


inline void display_rpy(const Eigen::MatrixXd& rpy_data) {
    char display_rpy_data;
    std::cout << "Display RPY Data? [y/n]: ";
    std::cin >> display_rpy_data;
    if(display_rpy_data == 'y' || display_rpy_data == 'Y'){
        std::cout << rpy_data << std::endl;
        std::cout << "        t        Missile_Roll     Missile_Pitch     Missile_Yaw        Target_Roll     Target_Pitch     Target_Yaw " << std::endl;
    }
    // Generates CSV / Plot of target angles
    char gen_csv;
    std::cout << "Generate CSV/Plot? [y/n]: ";
    std::cin >> gen_csv;
    if(gen_csv == 'y' || gen_csv == 'Y'){
        std::filesystem::create_directories("logs");
        SimLogger logger("logs/target_rpy.csv");
        for(int i = 0; i < rpy_data.rows(); ++i) {
            logger.log_rpy(rpy_data(i, 0), rpy_data.block<1,3>(i, 4).transpose());
        }
    }
}

inline void display_positions(const Eigen::MatrixXd& pos_log) {
    char display_pos;
    std::cout << "Display Full Data? [y/n]: ";
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
