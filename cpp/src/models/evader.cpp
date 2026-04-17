#include "flight_object_headers/evader.hpp"
#include "aero/aero.hpp"
#include <math.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>

Eigen::Matrix3d Evader::rotate_wind_to_body(const State& X, const Eigen::Vector3d C_aero, const AeroAngles A) const{
    // returns the rotation matrix from WIND -> BODY. 
    double c_a = cos(A.alpha);
    double c_b = cos(A.beta);
    double s_a = sin(A.alpha);
    double s_b = sin(A.beta);
    Eigen::Matrix3d R_wb{{c_a * c_b, s_b, s_a * c_b},
                    {-c_a * s_b, c_b, -s_a * s_b},
                    {-s_a, 0.0, c_a}};
    return R_wb;
}

State Evader::dXdt(const double T, const State& X) const{ // const function means it wont modify the variables
    /* 
    Position in inertial frame
    Attitude q as body relative to inertial (one quaternion, used for rotations)
    Body velocity v_b​ in body
    Body rates w_b​	in body 
    Wind from Wind to Body frame.
    */
    State dXdt = State();
    AeroAngles aero_angles = aero_func.recalc_aero_angles(X, wind_vel_i);
    Eigen::Matrix3d R_wb = rotate_wind_to_body(X, C_aero, aero_angles); // Rotation Matrix for wind frame -> Body frame.

    // ---- dXdt.q ---- (Converts Body to Inertial)
    Eigen::Quaterniond q_bi = X.q;    // Use Quaternion to convert body thrust to world frame
    Eigen::Quaterniond omega_q_b(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q_b);
    dXdt.q.coeffs() *= 0.5;
    // ---- dXdt.pos ---- (Inertial)
    dXdt.pos = q_bi * X.vel;
    // ---- Thrust ---- (Inertial)
    Eigen::Vector3d T_b(thrust,0,0);
    Eigen::Vector3d T_i = q_bi * T_b;
    // -- Velocities --
    const double v_mag = std::max((X.vel.norm()), 1e-6);
    Eigen::Vector3d v_hats = X.vel / v_mag;
    Eigen::Vector3d g_i(0,0,9.81);
    Eigen::Vector3d g_b = q_bi.conjugate() * g_i;
    // -- Aero Forces -- (Wind Frame)
    double dyn_pressure = 0.5 * density * v_mag * v_mag; // NOTE: Don't need to apply v_hats to L,D,Y as it is already encoded into the wind frame.
    double L = dyn_pressure * A * C_aero(0);
    double D = dyn_pressure * A * C_aero(1);
    double Y = dyn_pressure * A * C_aero(2);
    Eigen::Vector3d F_aero_w(-L, D, -Y);
    // -- Aero Forces (Body Frame) -- 
    Eigen::Vector3d F_aero_b = R_wb * F_aero_w;

    // ---- dXdt.vel ---- (Body)
    Eigen::Vector3d v_b = (1.0/m_evader) * (T_i + F_aero_b) - X.omega.cross(X.vel) + g_b;
    dXdt.vel = v_b;

    // ---- dXdt.omega ---- (Body)
    Eigen::Vector3d M_b = r_cp_cg.cross(F_aero_b); // - Moments (M) - 
    Eigen::Vector3d w = X.omega; // angular velocity
    Eigen::Vector3d ang_momentum = J * w;
    dXdt.omega = J.ldlt().solve(M_b + w.cross(ang_momentum));
    return dXdt;
}
