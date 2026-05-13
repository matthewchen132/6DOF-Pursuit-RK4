#pragma once
#include <vector>
#include <cmath>
#include <objects/State.hpp>
#include "aero/aero.hpp"

class flight_object{
    // Assuming no change in COM/mass over time (for now)
    private:
        aero_functions aerodynamics; // Aerodynamic helpers
        const double mass = 1000; // kg
        const double length = 10; // meters
        const double wingspan = 0.45; // meters
        const double MAC_chord_length = 0.2 * missile_l; // meters
        const double A = 1.0; //m^2
        const double air_density = 1.2754; // kg / m^3

        // -- Inertia --
        const double Jxx =0.5 * m_missile * r*r;
        const double Jyy = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        const double Jzz = (1.0/12.0) * m_missile * (3*r*r + missile_l*missile_l);
        // -- Inertia Tensor with Symmetry across the XZ Plane (Body) -- 
        const Eigen::Matrix3d J{{Jxx, 0.0, 0.0}, 
                            {0.0, Jyy, 0.0}, 
                            {0.0, 0.0, Jzz}}; //inertial matrix, x-z and                                            
    public:
        State X;
        State f(const double T, const State& X, Eigen::Vector3d a_norm) const;
        Eigen::Vector3d wind_vel_i;
        double thrust = 0.0; //N
};