#include "flight_object_headers/AMRAAM.hpp"
#include <cmath>
#include <iostream>

Eigen::Vector3d pro_nav_6dof(){
    
}

State AMRAAM::dXdt(double T, const State& X, Eigen::Vector3d a_norm) const { // NED
    State dXdt = State();
    // ---- dXdt.q ---- (rotate from body to world frame)
    Eigen::Quaterniond q_bi = X.q;
    Eigen::Quaterniond omega_q(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q);
    dXdt.q.coeffs() *= 0.5;

    // ---- dXdt.pos ----
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
    Eigen::Vector3d dynamic_p;
    dynamic_p.setOnes();
    dynamic_p = dynamic_p * 0.5 * density * v_mag * v_mag;
    // -- Drag --
    Eigen::Vector3d F_drag = dynamic_p * Cd * A * v_hats;
    // ---- dXdt.vel ---- (world)
    dXdt.vel = (T_w - F_drag)/(m_missile) - X.omega.cross(X.vel) + g_b + (a_norm * v_hats);
    // -- Moments -- (body)
    F_drag = dynamic_p * Cd * A;
    Eigen::Vector3d M_b =  r_cg_cp.cross(F_drag); 
    // ---- dXdt.omega ---- (body)
    Eigen::Vector3d w = X.omega;
    Eigen::Vector3d ang_accel = J * w;
    dXdt.omega= J.ldlt().solve(M_b - w.cross(ang_accel));
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