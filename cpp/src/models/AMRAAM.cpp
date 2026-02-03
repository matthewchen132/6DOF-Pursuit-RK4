#include "AMRAAM.hpp"
#include <cmath>
#include <iostream>

AMRAAM::State6 AMRAAM::dXdt_2d(double T, const State6& X_2d, double a_norm) const {
    const double v_mag = (X_2d(2) + X_2d(3) > 1e-9) ? std::hypot(X_2d(2), X_2d(3)) : 1e-9;
    const double u_portion = (v_mag > 1e-6) ? X_2d(2)/v_mag : 1e-6;
    const double v_portion = (v_mag > 1e-6) ? X_2d(3)/v_mag : 1e-6;
    State6 dXdt_2d = State6::Zero();
    dXdt_2d(0) = X_2d(2);
    dXdt_2d(1) = X_2d(3);
    dXdt_2d(2) = (thrust/m_missile)*cos(X_2d(4)) - ((u_portion)*0.5*Cd*A*density*v_mag*v_mag/m_missile) - a_norm*X_2d(3)/v_mag; // -x normal accel
    dXdt_2d(3) = (thrust/m_missile)*sin(X_2d(4)) - ((v_portion)*0.5*Cd*A*density*v_mag*v_mag/m_missile) + a_norm*X_2d(2)/v_mag; // + y normal accel
    dXdt_2d(4) = X_2d(5);
    dXdt_2d(5) = 0.0; // assumes constant headings
    return dXdt_2d;
}

PNresult AMRAAM::pro_nav_2d(double N, const State6& X_missile, const State6& X_targ){
    PNresult out;
    const Eigen::Vector2d r{
        X_targ(0) - X_missile(0),
        X_targ(1) - X_missile(1) 
    };
    const Eigen::Vector2d v_missile{
        X_missile(2), X_missile(3)
    };
    double mags_r_v = (r.norm()*v_missile.norm())
    double ang_missile_targ = acos(r.dot(v_missile)/mags_r_v); // angle between interceptor and target
    out.theta_los = std::clamp(ang_missile_targ, -1, 1); // angle between interceptor and target
    out.collided = (std::hypot(r(0), r(1)) < 1) ? true : false;
    const Eigen::Vector2d r_dot{
        X_targ(2) - X_missile(2),
        X_targ(3) - X_missile(3)
    };
    out.dLOS_dt = (r(0)*r_dot(1) - r(1)*r_dot(0))/r.squaredNorm();
    out.v_closing = -(r.dot(r_dot))/r.norm();
    out.a_norm = N * out.dLOS_dt * out.v_closing;
    return out; 
}

// bool pure_pursuit(double LOS_to_targ, auto lookahead_dist, auto L, const State6& X_missile, const State6& X_targ){
// // if LOS is big, use pure pursuit to get on track
// // 1) find LOS angle
// // 1a) if LOS angle outside of AMRAAM.fov, trigger pure pursuit.

//     // steer_angle_to_targ = atan2()
//     const Eigen::Vector2d r{
//         X_targ(0) - X_missile(0),
//         X_targ(1) - X_missile(1) 
//     };
//     const Eigen::Vector2d r_dot{
//         X_targ(2) - X_missile(2),
//         X_targ(3) - X_missile(3)
//     };
//     /*
//     if LOS_to_targ < fov
//         switch to pro_nav
//     */
     
// } 