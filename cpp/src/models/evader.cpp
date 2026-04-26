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
    State dXdt = State();
    AeroAngles aero_angles = aero_func.recalc_aero_angles(X, wind_vel_i);
    Eigen::Matrix3d R_wb = rotate_wind_to_body(X, C_aero, aero_angles); // Rotation Matrix for wind frame -> Body frame.

    // ---- dXdt.q ---- (Converts Body to Inertial)
    Eigen::Quaterniond q_bi = X.q;    // Use Quaternion to convert body thrust to world frame
    Eigen::Quaterniond omega_q_b(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q_b);
    dXdt.q.coeffs() *= 0.5;
    Eigen::Quaterniond q_ib = q_bi.conjugate();

    // ---- dXdt.pos ---- (Inertial)
    dXdt.pos = q_bi * X.vel;

    // ---- Thrust ---- (Body)
    Eigen::Vector3d T_b(thrust,0,0);

     // -- Gravity (World to Body) --
    const Eigen::Vector3d g_i(0.0, 0.0, 9.81);
    Eigen::Vector3d g_b = q_bi.conjugate() * g_i;

    // -- Dynamic Pressure -- (Wind Frame)
    Eigen::Vector3d wind_vel_b = q_ib * wind_vel_i;
    Eigen::Vector3d V_rel = X.vel - wind_vel_b;
    double v_mag = std::max(V_rel.norm(), 1e-6);
    Eigen::Vector3d v_hats = V_rel / v_mag;
    double dyn_pressure = 0.5 * density * v_mag * v_mag; // NOTE: Don't need to apply v_hats to L,D,Y as it is already encoded into the wind frame.

    // -- Aero Forces -- (Wind Frame)
    double L = dyn_pressure * A * C_aero(0);
    double D = dyn_pressure * A * C_aero(1);
    double Y = dyn_pressure * A * C_aero(2);
    Eigen::Vector3d F_aero_w(-D, Y, -L);

    // -- Aero Forces (Body Frame) -- 
    Eigen::Vector3d F_aero_b = R_wb * F_aero_w;

    // ---- dXdt.vel ---- (Body)
    Eigen::Vector3d v_b = (1.0/m_evader) * (T_b + F_aero_b) - X.omega.cross(X.vel) + g_b;
    dXdt.vel = v_b;

    // -- Moments -- (Body)
    Eigen::Vector3d M_b = r_cg_cp.cross(F_aero_b);  // must improve

    // ---- dXdt.omega ---- (Body)
    Eigen::Vector3d w = X.omega; // angular velocity
    Eigen::Vector3d ang_momentum = J * w;
    dXdt.omega = J.ldlt().solve(M_b - w.cross(ang_momentum));
    return dXdt;
}
