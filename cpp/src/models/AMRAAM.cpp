#include "flight_object_headers/AMRAAM.hpp"
#include <cmath>
#include <iostream>

Eigen::Matrix<double,6,1> AMRAAM::dXdt_2d(double T, const State& X, double a_norm) const {
    // first convert the acceleration magnitude into inertial frame commands
    Eigen::Vector2d v = {X.pos(0), X.pos(1)};
    const double v_mag = v.norm();
    Eigen::Vector2d v_components = v /v_mag; // components of velocity
    Eigen::Matrix<double,6,1> dXdt_2d;
    dXdt_2d(0) = X.vel(0); // vx
    dXdt_2d(1) = X.vel(1); // vy
    dXdt_2d(2) = (thrust/m_missile)*cos(X(4)) - ((v_components(0))*0.5*Cd*A*density*v_mag*v_mag/m_missile) - a_norm*X.vel(1)/v_mag; // -x normal accel
    dXdt_2d(3) = (thrust/m_missile)*sin(X(4)) - ((v_components(1))*0.5*Cd*A*density*v_mag*v_mag/m_missile) + a_norm*X.vel(0)/v_mag; // + y normal accel
    dXdt_2d(4) = X(5);
    dXdt_2d(5) = 0.0; // assumes constant headings
    return dXdt_2d;
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