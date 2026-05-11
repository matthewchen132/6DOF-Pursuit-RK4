#pragma once
#include "aero/aero.hpp"
#include <Eigen/Dense>

class Elevator{
    /*
     - Defines the Elevator Object -> Controls pitching moment of Fixed-wings
     1) Takes moment arm, Current Angular velocity and Pitching angle
    */
    public:
        Elevator(double moment_arm_length,
                AeroAngles aero_angles
                Eigen::Quaterniond q_bi, // Angular error
                Eigen::Quaterniond w_bi, // Angular Velocity (Gyroscope)
                Eigen::Matrix3d J // Inertial Tensor
                double )
        : 
        {}
        struct elevator_control{
            double Kp = 2.0;
            double q_cmd = 0.0; // pitch rate desired
            double delta_e = 0.0; // elevator deflection error
            const double delta_e_max_rad = 25.0 * M_PI /180.0;
        };
        double pitching_force(const AeroAngles& aero_angles) const; // Generates Force in the pitching Axis to apply.
        double deflection_cmd(const Eigen::Quaterniond& q_bi, const Eigen::Matrix3d& J) const; // Returns Deflection Angle.
    private:
        elevator_control control;


}