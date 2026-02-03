#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>


struct PNresult{ // data from Pro-nav
    bool collided;
    double dLOS_dt;
    double v_closing;
    double a_norm;
    double theta_los 
};

struct a_norm_vec2d{ //
    double a_nx;
    double a_ny;
};

struct a_norm_vec3d{ //
    double a_nx;
    double a_ny;
    double a_nz;
};

class AMRAAM{
    public:
        using State6 = Eigen::Matrix<double, 6, 1>;
        State6 X_2d = {0,0,100,0,0,0};
        State6 dXdt_2d(double T, const State6& X_2d, double a_norm) const;
        PNresult pro_nav_2d(double N, const State6& X_missile, const State6& X_targ);
        // bool pure_pursuit(double LOS_to_targ, auto lookahead_dist, L); // if LOS is big, use pure pursuit to get on track
    private:
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