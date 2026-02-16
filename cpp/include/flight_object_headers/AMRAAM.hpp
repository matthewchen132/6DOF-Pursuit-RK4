#pragma once

// #include <Eigen/Dense>
#include <vector>
#include <cmath>


struct PNresult{ // data from Pro-nav
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool collided;
    double dLOS_dt;
    double v_closing;
    double a_norm;
    double theta_los;
};



class AMRAAM{

struct State {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
    Eigen::Vector3d omega = Eigen::Vector3d::Zero();
    
    void normalize() {
        attitude.normalize();
    }
};
    public:
        State X;
        Eigen::Matrix<double,6,1> dXdt_2d(double T, const State& X, double a_norm) const;
        PNresult pro_nav_2d(double N, const State& X_missile, const State& X_targ, double collision_radius);
        // bool pure_pursuit(double LOS_to_targ, auto lookahead_dist, auto L);
    private:
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero(); //inertial matrix
        double m_missile = 1000; // kg
        double v_mag = 100.0; // m/s
        double thrust = 8000; //N
        double length = 10; // meters
        double r = .1; // meters
        double MOI = 0.5*(m_missile)*r*r*1.1; // (.5*MR^2)*1.1, will be turned into a 3x3 matrix of MOI (follow Wiley)
        double A = 1.0; //m^2
        double Cd = 0.08;
        double density = 1.2754; // kg / m^3
};