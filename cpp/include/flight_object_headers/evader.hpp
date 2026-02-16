#pragma once
#include <vector>
#include <cmath>

class Evader{

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
        State dXdt_2d(double T, const State& X_2d) const;


    private:
        Eigen::Matrix3d J = Eigen::Matrix3d::Zero();
        double m_evader = 1615; // kg
        double thrust = 5000; //N
        double length = 10; // meters
        double r = .1; // meters
        double MOI = 0.5*(m_evader)*r*r*1.1; // (.5*MR^2)*1.1, approximated with a cylinder, 
        double A = 1.0; //m^2
        double Cd = 0.08;
        double density = 1.2754; // kg / m^3
};