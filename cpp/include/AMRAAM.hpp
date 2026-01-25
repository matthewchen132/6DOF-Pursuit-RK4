#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

class AMRAAM{

    public:
        using State6 = Eigen::Matrix<double, 6, 1>;
        State6 X_2d = {0,0,0,0,0,0};
        // double phi = atan2(X[5], X[4]); // angle about X
        // double theta = atan2(X[5], X[3]); // angle about Y
        double psi = X_2d(4); // angle about Z
        std::vector<double> x_data, y_data;
        State6 dXdt_2d(double T, const State6& X_2d) const;
        State6 pro_nav(double Kp, const State6& X_Missile, const State6& X_Interceptor);


    private:
        double m_evader = 1615; // kg
        double thrust = 10000; //N
        double length = 10; // meters
        double r = .1; // meters
        double MOI = 0.5*(m_evader)*r*r*1.1; // (.5*MR^2)*1.1, approximated with a cylinder, 
        double A = 1.0; //m^2
        double Cd = 0.08;
        double density = 1.2754; // kg / m^3

};