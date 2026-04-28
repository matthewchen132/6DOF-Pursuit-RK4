#include "flight_object_headers/evader.hpp"
#include "aero/aero.hpp"
#include <math.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>

Eigen::Matrix3d Evader::rotate_wind_to_body(const AeroAngles A) const{
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
    AeroAngles aero_angles_w = aero_func.recalc_aero_angles(X, wind_vel_i);
    
    // ---- dXdt.q ---- (Converts Body to Inertial)
    Eigen::Quaterniond q_bi = X.q;    // Use Quaternion to convert body thrust to world frame
    Eigen::Quaterniond omega_q_b(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q_b);
    dXdt.q.coeffs() *= 0.5;
    Eigen::Quaterniond q_ib = q_bi.conjugate();
    
    // -- Aerodynamic Coefficients --
    AeroCoeffs C_aero;
    Eigen::Vector3d wind_vel_b = q_ib * wind_vel_i;
    Eigen::Vector3d V_rel = X.vel - wind_vel_b;
    C_aero.C_translational = aero_func.C_translations_f16(aero_angles_w);
    C_aero.C_moments = aero_func.C_moments_f16(X.omega, aero_angles_w, wingspan, MAC_chord_length, V_rel);
    Eigen::Matrix3d R_wb = rotate_wind_to_body(aero_angles_w); // Rotation Matrix for wind frame -> Body frame.
    
    // -- Dynamic Pressure -- (Wind Frame)
    const double v_mag = std::max(V_rel.norm(), 1e-6);
    Eigen::Vector3d v_hats = V_rel / v_mag;
    double dyn_pressure = 0.5 * density * v_mag * v_mag; // NOTE: Don't need to apply v_hats to L,D,Y as it is already encoded into the wind frame.
    
    // ---- dXdt.pos ---- (Inertial)
    dXdt.pos = q_bi * X.vel;
    
    // ---- Thrust ---- (Body)
    Eigen::Vector3d T_b(thrust,0,0);

     // -- Gravity (World to Body) --
    const Eigen::Vector3d g_i(0.0, 0.0, 9.81); 
    Eigen::Vector3d g_b = q_bi.conjugate() * g_i;

    // -- Aero Forces -- (Wind Frame)
    double L = dyn_pressure * ref_area * C_aero.C_translational(0);
    double D = dyn_pressure * ref_area * C_aero.C_translational(1);
    double Y = dyn_pressure * ref_area * C_aero.C_translational(2);
    Eigen::Vector3d F_aero_w(-D, Y, -L);
    Eigen::Vector3d F_aero_b = R_wb * F_aero_w; // Rotate to Body Frame
    
    // -- Aerodynamic Forces -- (Body Frame)
    Eigen::Vector3d v_b = (1.0/m_evader) * (T_b + F_aero_b) - X.omega.cross(X.vel) + g_b;
    dXdt.vel = v_b;

    // -- Moments -- (Body)
    Eigen::Vector3d M_aero_b;
    double M_aero_x = dyn_pressure * ref_area * C_aero.C_moments(0) * wingspan; 
    double M_aero_y = dyn_pressure * ref_area * C_aero.C_moments(1) * MAC_chord_length;
    double M_aero_z = dyn_pressure * ref_area * C_aero.C_moments(2) * wingspan;
    M_aero_b = Eigen::Vector3d(M_aero_x, M_aero_y, M_aero_z);
    Eigen::Vector3d M_b = M_aero_b; // Simple, assumed thrust acting through the COG. 
                                    // must improve to incorporate T and TVC Gimbal
    // ---- dXdt.omega ---- (Body)
    Eigen::Vector3d w = X.omega; // angular velocity
    Eigen::Vector3d ang_momentum = J * w;
    dXdt.omega = J.llt().solve(M_b - w.cross(ang_momentum));
    return dXdt;
}
