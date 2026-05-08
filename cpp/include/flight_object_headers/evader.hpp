#pragma once
#include <vector>
#include <cmath>
#include <State.hpp>
#include "aero/aero.hpp"
#include "quat_rpy/quaternion_utils.hpp"

class Evader{
    public:
        Evader(double m, double T_b, 
                double b, double c_bar, double l, 
                Eigen::Vector3d vel_i) 
                : 
                m_evader(m), thrust(T_b), 
                wingspan(b), MAC_chord_length(c_bar), length(l)
                {
                    ref_area = wingspan * MAC_chord_length;
                    Eigen::Vector3d r_cg_cp = Eigen::Vector3d(-length, 0.0, 0.0);
                    ref_area = wingspan * MAC_chord_length;
                    // Initialize Quaternion and Velocity
                    X.q_bi = initialize_quaternion(vel_i);
                    X.q_ib = X.q_bi.conjugate(); 
                    X.vel_b = {vel_i.norm(), 0.0, 0.0}; // Puts all velocity in body frame nose direction
                }
        State X;
        State f(const double T, const State& X) const;
        Eigen::Matrix3d rotate_wind_to_body(const AeroAngles A) const;
        Eigen::Vector3d wind_vel_i;
        
        const double Jxz = 1331.4132386;
        const double Jxx = 12820.614648;
        const double Jyy = 75673.623725;
        const double Jzz = 85552.113395;
        
        // -- Inertia Tensor with Symmetry across the XZ Plane (Body) -- 
        Eigen::Matrix3d J{{Jxx, 0.0, -Jxz},
        {0.0, Jyy, 0.0},
        {-Jxz, 0.0, Jzz}};     
    private:
        aero_functions aero_func; // Aerodynamic helpers
        double ref_area; // m^2
        const double m_evader; // kg
        const double thrust; // N
        const double wingspan; // meters
        const double MAC_chord_length; // meters
        const double length; // meters
        const double density = 1.2754; // kg / m^3
};