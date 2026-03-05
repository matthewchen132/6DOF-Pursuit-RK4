#pragma once   
#include <Eigen/Dense>
struct State {;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
};
// right mult
inline State operator*(const State& s, double c){
    State output;
    output.pos = s.pos * c;
    output.vel = s.vel * c;
    output.q.coeffs() = s.q.coeffs() * c;
    output.omega = s.omega * c;
    return output;
}
// left mult
inline State operator*(double c, const State& s){
    State output;
    output.pos = c * s.pos;
    output.vel = c * s.vel;
    output.q = c * s.q.coeffs();
    output.omega = c * s.omega;
    return output;
}
// addition
inline State operator+(const State& s1, const State& s2){
    State s_sum;
    s_sum.pos = s1.pos + s2.pos;
    s_sum.vel = s1.vel + s2.vel;
    s_sum.q = s1.q.coeffs() + s2.q.coeffs();
    s_sum.omega = s1.omega + s2.omega;
    return s_sum;
}