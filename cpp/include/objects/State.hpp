#pragma once   
#include <Eigen/Dense>

struct State {
    /* 
        b = body frame
        i = inertial frame
        w = wind frame
    */
    Eigen::Vector3d pos_i = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_b = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_ib = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_bi = Eigen::Quaterniond::Identity();
    Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();
};
inline State operator*(const State& s, double c){
    State output;
    output.pos_i = s.pos_i * c;
    output.vel_b = s.vel_b * c;
    output.q_ib.coeffs() = s.q_ib.coeffs() * c;
    output.q_bi.coeffs() = s.q_bi.coeffs() * c;
    output.omega_b = s.omega_b * c;
    return output;
}
inline State operator*(double c, const State& s){
    State output;
    output.pos_i = c * s.pos_i;
    output.vel_b = c * s.vel_b;
    output.q_ib = c * s.q_ib.coeffs();
    output.q_bi = c * s.q_bi.coeffs();
    output.omega_b = c * s.omega_b;
    return output;
}
inline State operator+(const State& s1, const State& s2){
    State s_sum;
    s_sum.pos_i = s1.pos_i + s2.pos_i;
    s_sum.vel_b = s1.vel_b + s2.vel_b;
    s_sum.q_ib = s1.q_ib.coeffs() + s2.q_ib.coeffs();
    s_sum.q_bi = s1.q_bi.coeffs() + s2.q_bi.coeffs();
    s_sum.omega_b = s1.omega_b + s2.omega_b;
    return s_sum;
}