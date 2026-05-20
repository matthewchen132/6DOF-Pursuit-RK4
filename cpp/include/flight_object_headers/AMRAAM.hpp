#pragma once
#include <objects/State.hpp>
#include <vector>
#include <cmath>
#include "aero/aero.hpp"
#include "quat_rpy/quaternion_utils.hpp"


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
    public:
        AMRAAM(double mass, double T_b, 
            double b, double c_bar, double l,
            Eigen::Vector3d vel_i) 
            :
            m_missile(mass), thrust(T_b),
            wingspan(b), MAC_chord_length(c_bar), missile_l(l) 
            {
                ref_area = wingspan * MAC_chord_length;
                // Initialize Quaternion and Velocity
                X.q_bi = initialize_quaternion(vel_i);
                X.q_ib = X.q_bi.conjugate(); 
                X.vel_b = {vel_i.norm(), 0.0, 0.0}; // Puts all velocity in body frame nose direction
            }
        State X;
        State f(const State& X, Eigen::Vector3d a_norm, Eigen::Vector3d wind_vel_i) const;
        PNresult pro_nav_2d(double N, const State& X_missile, const State& X_targ, double collision_radius) const;
        Eigen::Vector3d pro_nav_6dof(double N, const State& X_missile, const State& X_targ) const;
        Eigen::Vector3d pro_nav_6dof_ZEM(double N, const State& X_missile, const State& X_targ) const;
        double density = 1.2754; // kg / m^3
        const double cg_cp_length = 3.0; //m 
        const Eigen::Vector3d r_cg_cp = Eigen::Vector3d(-cg_cp_length, 0.0, 0.0); // moment arm is only in the pitching direction (body frame)
        Eigen::Matrix3d rotate_wind_to_body(const AeroAngles aero) const;
        AeroAngles get_aero_angles(const State& X, const Eigen::Vector3d& wind_vel_i) const {
            return aerodynamics.recalc_aero_angles(X, wind_vel_i);
        }

    private:
        aero_functions aerodynamics; // Aerodynamic helpers
        double ref_area; //m^2
        const double m_missile; // kg
        const double thrust; //N
        const double wingspan; // meters
        const double MAC_chord_length; // meters
        const double missile_l; // meters
        const double r = .1; // meters
        const double Jxx =0.5 * m_missile * r*r;
        const double Jyy = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        const double Jzz = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        
        // -- Inertia Tensor with Symmetry across the XZ Plane (Body) -- 
        const Eigen::Matrix3d J{{Jxx, 0.0, 0.0}, 
                            {0.0, Jyy, 0.0}, 
                            {0.0, 0.0, Jzz}}; //inertial matrix, x-z and                         
        
        // -- Coeff. of Drag, Lift, Sideforce --
        const Eigen::Matrix3d C{{Jxx, 0.0, 0.0}, 
                            {0.0, Jyy, 0.0}, 
                            {0.0, 0.0, Jzz}}; //inertial matrix, x-z and                      
};