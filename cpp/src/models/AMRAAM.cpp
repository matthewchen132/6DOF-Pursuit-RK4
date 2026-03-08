#include "flight_object_headers/AMRAAM.hpp"
#include <cmath>
#include <iostream>

Eigen::Vector3d AMRAAM::pro_nav_6dof(double N, const State& X_missile, const State& X_targ){
    Eigen::Vector3d r_tm = X_targ.pos - X_missile.pos; // Position: Target relative to missile.
    Eigen::Vector3d v_tm = X_targ.vel - X_missile.vel; // Velocity: Target relative to missile.
    double t_to_go = r_tm.norm()/v_tm.norm();
    Eigen::Vector3d ZEM_dist = r_tm + v_tm*t_to_go;
    // -- unit vectors in the radial (r) and normal (n) directions
    Eigen::Vector3d u_r = r_tm * (1 / r_tm.norm());
    // -- ZEM --
    Eigen::Vector3d ZEM_r = (ZEM_dist.dot(u_r)) * u_r;
    Eigen::Vector3d ZEM_n = ZEM_dist - ZEM_r;
    Eigen::Vector3d a_norm = N * ZEM_n / (t_to_go * t_to_go);
    // -- Kinematic Eqs --
    Eigen::Vector3d r_dot_missile = v_tm;
    Eigen::Vector3d v_dot_missile = a_norm;
    return a_norm;
}

State AMRAAM::dXdt(double T, const State& X, Eigen::Vector3d a_norm) const { // NED
    State dXdt = State();
    // ---- dXdt.q ---- (rotate from body to world frame)
    Eigen::Quaterniond q_bi = X.q;
    Eigen::Quaterniond omega_q(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q);
    dXdt.q.coeffs() *= 0.5;
    // ---- dXdt.pos ---- (World)
    dXdt.pos = q_bi * X.vel;
    // -- Thrust Body to Inertial --
    Eigen::Vector3d T_b(thrust, 0, 0);
    Eigen::Vector3d T_w = q_bi * T_b; // q T_b q^-1
    // -- Gravity (World to Body) --
    const Eigen::Vector3d g_w(0.0, 0.0, 9.81);
    Eigen::Vector3d g_b = q_bi.conjugate() * g_w; // the conjugate is q "inertial to body"
    // -- dynamic pressure --
    const double v_mag = std::max(X.vel.norm(), 1e-6);
    Eigen::Vector3d v_hats = X.vel / v_mag; // components of velocity
    double dyn_pressure = 0.5 * density * v_mag * v_mag;
    // -- Drag -- (body)
    Eigen::Vector3d F_drag = -dyn_pressure * Cd * A * v_hats;
    // -- Moments -- (body)
    Eigen::Vector3d M_b =  r_cg_cp.cross(F_drag);  // must improve
    // ---- dXdt.vel ---- (body)
    Eigen::Vector3d v_b = (1.0/m_missile) * (T_w + F_drag) - X.omega.cross(X.vel) + g_b + (a_norm);
    dXdt.vel = v_b;
    // ---- dXdt.omega ---- (body)
    Eigen::Vector3d w = X.omega;
    Eigen::Vector3d ang_momentum = J * w;
    dXdt.omega= J.ldlt().solve(M_b - w.cross(ang_momentum));
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