#pragma once
#include <State.hpp>
#include <vector>
#include <cmath>
#include "aero/aero.hpp"


struct PNresult{ // data from Pro-nav
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool collided;
    double dLOS_dt;
    double v_closing;
    double a_norm;
    double theta_los;
};

class AMRAAM{
    // Assuming no change in COM/mass over time (for now)
    private:
        aero_functions aero_func; // Aerodynamic helpers
        const double m_missile = 1000; // kg
        const double v_mag = 100.0; // m/s
        const double missile_l = 10; // meters
        const double r = .1; // meters
        const double MOI = 0.5 * m_missile * r*r *1.1; // (.5*MR^2)*1.1, will be turned into a 3x3 matrix of MOI (follow Wiley)
        const double Jxx =0.5 * m_missile * r*r;
        const double Jyy = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        const double Jzz = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        // -- Inertia Tensor with Symmetry across the XZ Plane (Body) -- 
        Eigen::Matrix3d J{{Jxx, 0.0, 0.0}, 
                            {0.0, Jyy, 0.0}, 
                            {0.0, 0.0, Jzz}}; //inertial matrix, x-z and                         
        // -- Coeff. of Drag, Lift, Sideforce --
        Eigen::Matrix3d C{{Jxx, 0.0, 0.0}, 
                            {0.0, Jyy, 0.0}, 
                            {0.0, 0.0, Jzz}}; //inertial matrix, x-z and                      
    public:
        State X;
        State dXdt(const double T, const State& X, Eigen::Vector3d a_norm) const;
        PNresult pro_nav_2d(double N, const State& X_missile, const State& X_targ, double collision_radius) const;
        Eigen::Vector3d pro_nav_6dof(double N, const State& X_missile, const State& X_targ) const;
        Eigen::Vector3d wind_vel_i;
        double thrust = 17000; //N
        double A = 1.0; //m^2
        Eigen::Vector3d C_aero = Eigen::Vector3d (0.08, 0.16, 0.4);    
        double density = 1.2754; // kg / m^3
        const double length = 3.0; //m 
        const Eigen::Vector3d r_cg_cp = Eigen::Vector3d(-length, 0.0, 0.0); // moment arm is only in the pitching direction (body frame)
        Eigen::Matrix3d rotate_wind_to_body(const State& X, const Eigen::Vector3d C_aero, const AeroAngles aero) const;                      
};