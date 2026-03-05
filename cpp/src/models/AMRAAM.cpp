#include "flight_object_headers/AMRAAM.hpp"
#include <cmath>
#include <iostream>

// Eigen::Matrix<double,6,1> AMRAAM::dXdt_2d(double T, const State& X, double a_norm) const {
//     Eigen::Vector2d v = {X.pos(0), X.pos(1)};
//     const double v_mag = v.norm();
//     Eigen::Vector2d v_components = v /v_mag; // components of velocity
//     Eigen::Matrix<double,6,1> dXdt_2d;
//     dXdt_2d(0) = X.vel(0); // vx
    
//     dXdt_2d(1) = X.vel(1); // vy
//     dXdt_2d(2) = (thrust/m_missile)*cos(X(4)) - ((v_components(0))*0.5*Cd*A*density*v_mag*v_mag/m_missile) - a_norm*X.vel(1)/v_mag; // -x normal accel
//     dXdt_2d(3) = (thrust/m_missile)*sin(X(4)) - ((v_components(1))*0.5*Cd*A*density*v_mag*v_mag/m_missile) + a_norm*X.vel(0)/v_mag; // + y normal accel
//     dXdt_2d(4) = X(5);
//     dXdt_2d(5) = 0.0; // assumes constant headings
//     return dXdt_2d;
// }

State AMRAAM::dXdt(double T, const State& X, double a_norm) const { // NED
    State dXdt = State();
    // ---- dXdt.pos ----
    dXdt.pos = X.vel;
    // ---- dXdt.q ---- (rotate from body to world frame)
    Eigen::Quaterniond q = X.q.normalized(); // creates a copy of X.q
    Eigen::Quaterniond omega_q(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q * omega_q);
    dXdt.q.coeffs() *= 0.5;
    // -- Thrust Body to Inertial --
    Eigen::Vector3d T_b(thrust, 0, 0);
    Eigen::Vector3d T_ned = q * T_b; // q T_b q^-1
    // ---- dXdt.vel ---- (world)
    const double v_mag = std::max(X.vel.norm(), 1e-6);
    Eigen::Vector3d v_hats = X.vel / v_mag; // components of velocity
    double dynamic_p = 0.5 * density * v_mag * v_mag;
    dXdt.vel(0) = (T_ned(0)/m_missile) - ((v_hats(0))*Cd*A * dynamic_p /m_missile) - a_norm*X.vel(0)/v_mag; // x normal accel
    dXdt.vel(1) = (T_ned(1)/m_missile) - ((v_hats(1))*Cd*A * dynamic_p /m_missile) - a_norm*X.vel(1)/v_mag; // y normal accel
    dXdt.vel(2) = (T_ned(2)/m_missile) - ((v_hats(2))*Cd*A * dynamic_p /m_missile) - a_norm*X.vel(2)/v_mag + 9.81; // z normal accel
    // ---- dXdt.omega ----
    dXdt.omega(0) = 0.0;
    dXdt.omega(1) = 0.0;
    dXdt.omega(2) = -((0.5*Cd*A*density*v_mag*v_mag) * missile_l) / Jzz;
    return dXdt;
}

PNresult AMRAAM::pro_nav_2d(const double N, const State& X_missile, const State& X_targ, double collision_radius){
    PNresult out;
    const Eigen::Vector2d r{
        X_targ.pos(0) - X_missile.pos(0),
        X_targ.pos(1) - X_missile.pos(1) 
    };
    const Eigen::Vector2d v_missile{
        X_missile.vel(0), X_missile.vel(1)
    };
    out.collided = (std::hypot(r(0), r(1)) <= collision_radius) ? true : false;
    const Eigen::Vector2d r_dot{
        X_targ.vel(0) - X_missile.vel(0),
        X_targ.vel(1) - X_missile.vel(1)
    };
    if(r.squaredNorm() <= 1e-9){
        out.dLOS_dt = 0.0;
        out.v_closing = 0.0;
        out.a_norm = 0.0;
        return out;
    }

    out.dLOS_dt = (r(0)*r_dot(1) - r(1)*r_dot(0))/r.squaredNorm();
    out.v_closing = -(r.dot(r_dot))/r.norm();
    out.a_norm = N * out.dLOS_dt * out.v_closing;
    return out; 
}