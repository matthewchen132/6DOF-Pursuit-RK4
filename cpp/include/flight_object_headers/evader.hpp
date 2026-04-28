#pragma once
#include <vector>
#include <cmath>
#include <State.hpp>
#include "aero/aero.hpp"

class Evader{
    public:
        Evader(double m, double T_b, 
                double b, double c_bar, double l) 
                : 
                m_evader(m), thrust(T_b), 
                wingspan(b), MAC_chord_length(c_bar), length(l) 
                {
                    ref_area = wingspan * MAC_chord_length;
                    Eigen::Vector3d r_cg_cp = Eigen::Vector3d(-length, 0.0, 0.0);
                }
        State X;
        State dXdt(const double T, const State& X) const;
        Eigen::Matrix3d rotate_wind_to_body(const AeroAngles A) const;
        Eigen::Vector3d wind_vel_i;
        
        Eigen::Vector3d C_aero = Eigen::Vector3d (0.00, 0.0, 0.0);    
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