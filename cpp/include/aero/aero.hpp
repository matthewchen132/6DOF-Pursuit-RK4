#pragma once
#include <Eigen/Dense>
#include <cmath>

struct AeroCoeffs{
    double CL, CD, CY; // Sideforce
    double Cl, Cm, Cn; // Yawing Moment (Body Z)
};

struct AeroAngles{
    double alpha; // Angle of attack
    double beta; // Sideslip
};

class aero_functions{
    // recalc_aero_coefficients(double wingspan, double mean_aerodynamic_chord, Eigen::Vector3d& w_b, double& alpha, double& beta)
    public:
        AeroAngles recalc_aero_angles(const State& X, const Eigen::Vector3d& wind_vel) const{
            AeroAngles A;

            // -- Convert q_bi to q_ib --
            Eigen::Quaterniond q_ib = X.q.conjugate();
            q_ib.normalize();

            // -- Get the relative wind velocity --
            Eigen::Vector3d wind_vel_b = q_ib * wind_vel;
            Eigen::Vector3d V_rel_b = X.vel - wind_vel_b;
            double u = V_rel_b(0);
            double v = V_rel_b(1);
            double w = V_rel_b(2);
            double V_mag = std::hypot(u,v,w);

            if (V_mag < 1e-6) {
                A.alpha = 0.0;
                A.beta  = 0.0;
                return A;
            }
            
            // -- Set Angles --
            A.alpha = std::atan2(w, u);
            A.beta = std::asin(v/V_mag);
            return A;
        }
};