#pragma once
#include <stdio.h>
#include <initializer_list>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <stdio.h>
#include <vector>
#include <objects/State.hpp>
#include <cmath>

inline Eigen::Quaterniond initialize_quaternion(const Eigen::Vector3d& v_i) {
    /*
    This function generates the quaternion (q_bi) to bring the orientation of our object (Inertial) to the Body frame.
    Returns:
        q_bi: quaternion representing Body -> Inertial rotation
        Initially sets a roll angle of 0.0.
    */
    double yaw = std::atan2(v_i.y(), v_i.x());
    double pitch = std::atan2(-v_i.z(), v_i.head<2>().norm());
    
    // Apply the Yaw rotation and pitch rotation
    Eigen::Vector3d unit_b = Eigen::Vector3d::UnitX(); //  Unit vector in inertial frame (1,0,0)
    Eigen::AngleAxisd R_z(yaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd R_y(pitch, Eigen::Vector3d::UnitY());

    Eigen::Quaterniond q_bi = Eigen::Quaterniond(R_z) * Eigen::Quaterniond(R_y);
    q_bi.normalize();

    return q_bi;
}

inline Eigen::Vector3d quaternion_to_euler_ZYX(const Eigen::Quaterniond& q){
    /*
    Returns Euler Angles in order of z, y, x (yaw, pitch, roll)
    radians.
    Handles gimbal lock at pitch = ±90° by setting roll = 0 and recovering
    yaw from the remaining coupled degree of freedom.
    */
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d euler_zyx;

    double sin_pitch = std::clamp(-R(2,0), -1.0, 1.0);

    if (std::abs(sin_pitch) > 1.0 - 1e-6) {
        // Gimbal lock: cos(pitch) ≈ 0, roll and yaw are indistinguishable.
        // Set roll = 0 by convention and absorb all rotation into yaw.
        euler_zyx(1) = std::copysign(M_PI / 2.0, sin_pitch);
        euler_zyx(2) = 0.0;
        euler_zyx(0) = (sin_pitch > 0.0) ? std::atan2(-R(0,1), R(0,2))   // pitch = +90
                                          : std::atan2(-R(0,1), R(1,1));  // pitch = -90
    } else {
        euler_zyx(0) = std::atan2(R(1,0), R(0,0));  // yaw
        euler_zyx(1) = std::asin(sin_pitch);          // pitch
        euler_zyx(2) = std::atan2(R(2,1), R(2,2));   // roll
    }

    return euler_zyx;
}