#pragma once
#include "aero/aero.hpp"
#include <Eigen/Dense>

class Elevator{
    /*
     - Defines the Elevator Object -> Controls pitching moment of Fixed-wings
     1) Takes moment arm, Current Angular velocity and Pitching angle
    */
    public:
        Elevator(const Eigen::Vector3d& moment_arm_length,
                const AeroAngles& aero_angles,
                const Eigen::Quaterniond& q_bi, // Angular error
                const Eigen::Quaterniond& w_bi, // Angular Velocity (Gyroscope)
                const Eigen::Matrix3d& J,
                const Eigen::Vector3d& M_aero_b,
                const Eigen::Vector3d& accel_b,
                const double weight) // Inertial Tensor
                : 
                l(moment_arm_length), alpha_beta(aero_angles),
                q_bi(q_bi), w_bi(w_bi), J(J), 
                M_aero_b(M_aero_b), accel_b(accel_b), weight(weight)
                {}
        struct pitch_trim{
            double alpha = 0.0;
            double V_term = 0.0;
            double elevator_angle = 0.0;
            double pitch_angle = 0.0;
        };
        struct elevator_control{
            double Kp = 2.0;
            double q_cmd = 0.0; // pitch rate desired
            double delta_e = 0.0; // elevator deflection error
            const double delta_e_max_rad = 25.0 * M_PI /180.0;
        };
        double pitching_force(const AeroAngles& aero_angles) const; // Generates Force in the pitching Axis to apply.
        double deflection_cmd(const Eigen::Quaterniond& q_bi, const Eigen::Matrix3d& J) const; // Returns Deflection Angle.
            // Sum of Pitching Moment = 0
            // Sum of Pitching Angular Velocity = 0
            // Sum of Accelerations = 0
            // F_Lift - weight = 0.0
        pitch_trim find_trim(const State& X, const Eigen::Matrix3d& J, 
                            const double accel_x_b, const double M_pitch) const; // creates a fresh pitch_trim
        Eigen::Vector3d l;
        AeroAngles alpha_beta;
        Eigen::Quaterniond q_bi; // Angular error
        Eigen::Quaterniond w_bi; // Angular Velocity (Gyroscope)
        Eigen::Matrix3d J;
        Eigen::Vector3d M_aero_b;
        Eigen::Vector3d accel_b;
        double Cm_e;
        double elevator_angle;
        double weight;
    private:
        elevator_control control;
};