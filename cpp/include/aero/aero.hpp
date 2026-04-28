#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <assert.h>

struct AeroCoeffs{
    // Translational Aerodynamic Coefficients
    Eigen::Vector3d C_translational = Eigen::Vector3d(0.0, 0.0, 0.0); // CL, CD, CY
    
    // Moment Aerodynamic Coefficients
    Eigen::Vector3d C_moments = Eigen::Vector3d(0.0, 0.0, 0.0); // C_l, C_m, C_r
};

struct AeroAngles{
    double alpha; // Angle of attack
    double beta; // Sideslip
};

class aero_functions{
    public:
        AeroAngles recalc_aero_angles(const State& X, const Eigen::Vector3d& wind_vel) const{
            /*
                Returns an AeroAngles object with alpha (angle of attack) and beta (angle of sideslip)
            */
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

        Eigen::Vector3d C_moments_missile(const Eigen::Vector3d& omega, const AeroAngles& aero_angles, 
                                        const double wingspan, const double MAC_chord_length, const Eigen::Vector3d& V_rel) const {       
            /*
                Returns an Eigen::Vector3d representing translational force
            */
            Eigen::Vector3d C_m;
            double V = std::max(V_rel.norm(), 1e-6);
            double alpha = aero_angles.alpha;
            double beta = aero_angles.beta;
            double b = wingspan;
            double c_bar = MAC_chord_length;
            
            // -- Moments Simplified --
            double Cl_0 = 0.0, Cl_beta = -0.1, Cl_p = -0.5;
            double Cm_0 = 0.0, Cm_alpha = -1.0, Cm_q = -0.25;
            double Cn_0 = 0.0, Cn_beta = 0.1, Cn_r = -0.2;

            assert((Cl_p < 0) && "Cl_p must be negative");
            assert((Cm_q < 0) && "Cm_q must be negative");
            assert((Cn_r < 0) && "Cn_r must be negative");

            // scaling for damping aero moments
            double p_hat = (b/(2*V)) * omega(0);
            double q_hat = (c_bar/(2*V)) * omega(1);
            double r_hat = (b/(2*V)) * omega(2);
            C_m(0) = Cl_0 + (Cl_beta*beta) + Cl_p * p_hat; // rolling moment (C_l)
            C_m(1) = Cm_0 + (Cm_alpha*alpha) + Cm_q * q_hat; // pitch moment (C_m)
            C_m(2) = Cn_0 + (Cn_beta*beta) + Cn_r * r_hat; // yaw moment (C_r)

            return C_m;
        }

        Eigen::Vector3d C_translations_missile(const AeroAngles& aero_angles) const {
            /*
                Returns an Eigen::Vector3d representing translational force
            */
            Eigen::Vector3d C;

            double alpha = aero_angles.alpha;
            double beta  = aero_angles.beta;

            double CL0 = 0.0, CL_alpha = 5.0;
            double CD0 = 0.02, CD_alpha2 = 0.3;
            double CY_beta = -0.98;

            double CL = CL0 + CL_alpha * alpha;
            double CD = CD0 + CD_alpha2 * alpha * alpha;
            double CY = CY_beta * beta;

            C << CL, CD, CY;

            return C;
        }

        Eigen::Vector3d C_moments_f16(const Eigen::Vector3d& omega, const AeroAngles& aero_angles, 
                                        const double wingspan, const double MAC_chord_length, const Eigen::Vector3d& V_rel) const {       
            /*
                Returns an Eigen::Vector3d representing translational force
            */
            Eigen::Vector3d C_m;
            double V = std::max(V_rel.norm(), 1e-6);
            double alpha = aero_angles.alpha;
            double beta = aero_angles.beta;
            double b = wingspan;
            double c_bar = MAC_chord_length;
            
            // -- Moments Simplified --
            double Cl_0     = 0.0;
            double Cl_beta  = -0.12;   // roll due to sideslip (dihedral effect)
            double Cl_p     = -0.5;    // roll damping

            double Cm_0     = 0.05;
            double Cm_alpha = -0.8;    // weaker stability (relaxed stability aircraft)
            double Cm_q     = -10.0;   // STRONG pitch damping

            double Cn_0     = 0.0;
            double Cn_beta  = 0.25;    // strong weathercock stability
            double Cn_r     = -0.35;   // yaw damping

            assert((Cl_p < 0) && "Cl_p must be negative");
            assert((Cm_q < 0) && "Cm_q must be negative");
            assert((Cn_r < 0) && "Cn_r must be negative");

            // scaling for damping aero moments
            double p_hat = (b/(2*V)) * omega(0);
            double q_hat = (c_bar/(2*V)) * omega(1);
            double r_hat = (b/(2*V)) * omega(2);
            C_m(0) = Cl_0 + (Cl_beta*beta) + Cl_p * p_hat; // rolling moment (C_l)
            C_m(1) = Cm_0 + (Cm_alpha*alpha) + Cm_q * q_hat; // pitch moment (C_m)
            C_m(2) = Cn_0 + (Cn_beta*beta) + Cn_r * r_hat; // yaw moment (C_r)

            return C_m;
        }

        Eigen::Vector3d C_translations_f16(const AeroAngles& aero_angles) const {
            /*
                Returns an Eigen::Vector3d representing translational force
            */
            Eigen::Vector3d C;

            double alpha = aero_angles.alpha;
            double beta  = aero_angles.beta;

            // -- F-16 Coeff. Approximation  --
            double CL0        = 0.2;
            double CL_alpha   = 4.5;    // lift slope (~4–6 per rad typical)

            double CD0        = 0.02;
            double CD_alpha2  = 0.3;    // quadratic drag growth

            double CY_beta    = -0.98;  // sideforce from sideslip

            double CL = CL0 + CL_alpha * alpha;
            double CD = CD0 + CD_alpha2 * alpha * alpha;
            double CY = CY_beta * beta;

            C << CL, CD, CY;

            return C;
        }
    };
