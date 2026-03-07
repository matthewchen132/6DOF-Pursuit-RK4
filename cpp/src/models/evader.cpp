// #include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <string>
#include "flight_object_headers/evader.hpp"
#include "Common.hpp"


State Evader::dXdt(double T, const State& X) const{ // const function means it wont modify the variables
   
    /* 
    Position in inertial/NED
    Attitude q as body relative to inertial (one quaternion, used for rotations)
    Body velocity v_b​ in body
    Body rates w_b​	in body 
    */
    State dXdt = State();
    // ---- dXdt.q ---- (Converts Body to Inertial)
    Eigen::Quaterniond q_b = X.q;    // Use Quaternion to convert body thrust to world frame
    Eigen::Quaterniond omega_q_b(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_b * omega_q_b);
    dXdt.q.coeffs() *= 0.5;
    // ---- dXdt.pos ---- (Inertial)
    dXdt.pos = q_b * X.vel;
    // ---- Thrust ---- (Inertial)
    Eigen::Vector3d T_b(thrust,0,0);
    Eigen::Vector3d T_ned = q_b * T_b;
    // -- Velocities --
    const double v_mag = std::max((X.vel.norm()), 1e-6);
    Eigen::Vector3d v_hats = X.vel / v_mag;
    Eigen::Vector3d g_i(0,0,9.81);
    Eigen::Vector3d g_b = q_b.conjugate() * g_i;
    // -- Drag --
    double dyn_pressure = 0.5 * density * v_mag * v_mag;
    Eigen::Vector3d F_drag(dyn_pressure * Cd * A * v_hats(0), 
                           dyn_pressure * Cd * A * v_hats(1), 
                           dyn_pressure * Cd * A * v_hats(2));
    // ---- dXdt.vel ---- (Body)
    Eigen::Vector3d vel_b = (1.0/m_evader) * (T_ned - F_drag) - dXdt.omega.cross(dXdt.vel) + g_b;
    // ---- dXdt.omega ---- (Body)
    Eigen::Vector3d M_b = F_drag * length; // - Moments (M) - 
    Eigen::Vector3d w = X.omega; // angular velocity
    Eigen::Vector3d ang_accel = J * w;
    dXdt.omega = J.ldlt().solve(M_b - w.cross(ang_accel));
    

    /* current goals:
     
    1) current issue with omega: implement a controller to keep me from rotating like crazy
    2) this is a good time to implement wind
    3) implement onto Missile interceptor as well.
    */
    return dXdt;
}