#pragma once
#include "aero/aero.hpp"
#include <Eigen/Dense>
#include "control_tools/pitch_trim.hpp"

class ElevatorControl{
    public:
        ElevatorControl() = default;
        ElevatorControl(const double Kp, const double Kd,
            const double q_cmd, const double tau, const double actuator_limit)
            :
            Kp(Kp), Kd(Kd), q_cmd(q_cmd), tau(tau),
            delta_e_max_rad(actuator_limit)
        {}
        double get_Kp()    const { return Kp; }
        double get_Kd()    const { return Kd; }
        double get_q_cmd() const { return q_cmd; }
        double get_limit() const { return delta_e_max_rad; }
        double get_tau()   const { return tau; }
    private:
        double Kp              = 0.0;
        double Kd              = 0.0;
        double q_cmd           = 0.0;
        double tau             = 0.0495;
        double delta_e_max_rad = 25.0 * M_PI / 180.0;
};

class Elevator{
    public:
        Elevator() = default;
        Elevator(const Eigen::Vector3d& moment_arm_length,
            const AeroAngles& aero_angles,
            const Eigen::Quaterniond& q_bi,
            const Eigen::Vector3d& omega_bi,
            const Eigen::Matrix3d& J,
            const ElevatorControl& elevator_controller,
            const double weight)
            :
            l(moment_arm_length), alpha_beta(aero_angles),
            q_bi(q_bi), omega_bi(omega_bi), J(J),
            weight(weight), control(elevator_controller)
            {}
        // pitch_rate_rads: pass X.omega_b(1) from current state
        double delta_cmd(const ElevatorControl& control, double pitch_rate_rads) const {
            return control.get_Kp() * (control.get_q_cmd() - pitch_rate_rads);
        }
        double delta_actual(const ElevatorControl& control, const double delta_cmd, const double dt){
            double delta_e_dot = (delta_cmd - delta_e) / control.get_tau();
            delta_e_dot = std::clamp(delta_e_dot, -60.0 * M_PI/180.0, 60.0 * M_PI/180.0);
            delta_e += delta_e_dot * dt;
            delta_e = std::clamp(delta_e, -control.get_limit(), control.get_limit());
            return delta_e;
        }
        Eigen::Vector3d l;
        AeroAngles alpha_beta;
        Eigen::Quaterniond q_bi;
        Eigen::Vector3d omega_bi;
        Eigen::Matrix3d J;
        Eigen::Vector3d M_aero_b;
        Eigen::Vector3d accel_b;
        double Cm_e    = -0.8;
        double weight  = 0.0;
        double delta_e = 0.0;
    private:
        ElevatorControl control;
};
