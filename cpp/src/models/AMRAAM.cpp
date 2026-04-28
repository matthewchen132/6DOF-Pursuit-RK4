#include "flight_object_headers/AMRAAM.hpp"
#include "aero/aero.hpp"
#include "simulation.hpp"
#include <cmath>
#include <iostream>
#include <algorithm>

Eigen::Matrix3d AMRAAM::rotate_wind_to_body(const AeroAngles aero) const{
    // returns the rotation matrix from WIND -> BODY. 
    double c_a = cos(aero.alpha);
    double c_b = cos(aero.beta);
    double s_a = sin(aero.alpha);
    double s_b = sin(aero.beta);
    Eigen::Matrix3d R_wb{{c_a * c_b, s_b, s_a * c_b},
                    {-c_a * s_b, c_b, -s_a * s_b},
                    {-s_a, 0.0, c_a}};
    return R_wb;
}
Eigen::Vector3d AMRAAM::pro_nav_6dof(double N, const State& X_missile, const State& X_targ) const{
    /*
    Full 6-DOF Proportional Navigation using LOS rate. More Robust than ZEM to t_to_go explosion.
    */  
    Eigen::Vector3d a_norm_cmd;
    
    // -- r_hat -- 
    Eigen::Vector3d r_tm = X_missile.pos - X_targ.pos;
    double r_mag = r_tm.norm();
    Eigen::Vector3d r_hat = r_tm.normalized();
    
    // -- LOS_rate --
    Eigen::Vector3d V_tm_i = X_missile.q * X_missile.vel - X_targ.q * X_targ.vel;
    Eigen::Vector3d LOS_rate = r_hat.cross(V_tm_i) / r_mag;
    
    // -- V_closing (Inertial) --
    double V_closing = -r_hat.dot(V_tm_i);

    a_norm_cmd = N * LOS_rate * V_closing;
    a_norm_cmd = actuator_limit(a_norm_cmd, 200);
    return a_norm_cmd;
}

Eigen::Vector3d AMRAAM::pro_nav_6dof_ZEM(double N, const State& X_missile, const State& X_targ) const{
    /*
    Full 6-DOF Proportional Navigation using ZEM.
     - Susceptible to t_to_go explosion due to V_closing rates. 
    * IF USING, MAKE SURE TO PROPERLY RESTRICT ORIENTATIONS TO PREVENT SMALL V_Closing.
    */
    Eigen::Vector3d r_tm = X_targ.pos - X_missile.pos; // Target relative to missile.
    if (r_tm.norm() == 0){
        r_tm.fill(1e-6); // Fill r_tm with values, norm = 1e-6
    }
    Eigen::Quaterniond q_bi_missile = X_missile.q;
    Eigen::Quaterniond q_bi_targ = X_targ.q;
    Eigen::Vector3d v_tm_i = q_bi_targ * X_targ.vel - q_bi_missile * X_missile.vel; // Inertial-frame relative velocity
    if(v_tm_i.norm() == 0){
        v_tm_i.fill(1e-6); // Fill v_tm with values, norm = 1e-6
    }

    // Problem: Small closing velocity -> high t_to_go -> 0 a_norm_cmd
    double V_closing = -r_tm.normalized().dot(v_tm_i);

    double t_to_go = r_tm.norm() / std::max(V_closing, 1e-6);
    Eigen::Vector3d ZEM_dist = r_tm + v_tm_i * t_to_go;

    // -- unit vectors in the radial (r) / normal (n) directions
    Eigen::Vector3d u_r = r_tm * (1 / r_tm.norm());

    // -- ZEM --
    Eigen::Vector3d ZEM_r = (ZEM_dist.dot(u_r)) * u_r;
    Eigen::Vector3d ZEM_n = ZEM_dist - ZEM_r;
    Eigen::Vector3d a_norm_cmd = N * ZEM_n / (t_to_go * t_to_go);
    return a_norm_cmd;
}

State AMRAAM::dXdt(const double T, const State& X, Eigen::Vector3d a_norm_i) const { // NED
    State dXdt = State();
    AeroAngles aero_angles = aerodynamics.recalc_aero_angles(X, wind_vel_i);

    // -- dXdt.q -- (Inertial)
    Eigen::Quaterniond q_bi = X.q; // X.q is body to inertial
    Eigen::Quaterniond omega_q(0.0, X.omega(0), X.omega(1), X.omega(2));
    dXdt.q = (q_bi * omega_q);
    dXdt.q.coeffs() *= 0.5;
    Eigen::Quaterniond q_ib = q_bi.conjugate();

    // -- Aerodynamic Coefficients -- (Wind Frame)
    AeroCoeffs C_aero;
    const Eigen::Vector3d wind_vel_b = q_ib * wind_vel_i;
    Eigen::Vector3d V_rel = X.vel - wind_vel_b;
    C_aero.C_translational = aerodynamics.C_translations_missile(aero_angles);
    C_aero.C_moments = aerodynamics.C_moments_missile(X.omega, aero_angles, wingspan, MAC_chord_length, V_rel);
    Eigen::Matrix3d R_wb = rotate_wind_to_body(aero_angles); // Rotation Matrix: Wind -> Body 
        
    // -- Dynamic Pressure -- (Wind Frame)
    const double v_mag = std::max(V_rel.norm(), 1e-6);
    Eigen::Vector3d v_hats = V_rel / v_mag;
    double dyn_pressure = 0.5 * density * v_mag * v_mag;
    
    // -- dXdt.pos -- (World)
    dXdt.pos = q_bi * X.vel;

    // -- Thrust -- (Body)
    Eigen::Vector3d T_b(thrust, 0, 0);
    

    // -- Gravity (World to Body) --
    const Eigen::Vector3d g_i(0.0, 0.0, 9.81);
    Eigen::Vector3d g_b = q_ib * g_i; // -- Body Frame --
    
    // -- Aerodynamic Forces -- (Wind Frame)
    double L = dyn_pressure * ref_area * C_aero.C_translational(0);
    double D = dyn_pressure * ref_area * C_aero.C_translational(1);
    double Y = dyn_pressure * ref_area * C_aero.C_translational(2);
    Eigen::Vector3d F_aero_w(-D, Y, -L);
    
    // -- Aerodynamic Forces -- (Body Frame)
    Eigen::Vector3d F_aero_b = R_wb * F_aero_w;
    
    // -- Convert a_norm to body frame, zero component in nose direction --
    Eigen::Vector3d a_norm_b = q_ib * a_norm_i;
    Eigen::Vector3d F_guidance_b = m_missile * a_norm_b; // Point-mass application
    
    // ---- dXdt.vel ---- (Body Frame)
    Eigen::Vector3d V_b = (1.0/m_missile) * (T_b + F_aero_b + F_guidance_b) - X.omega.cross(X.vel) + g_b;
    dXdt.vel = V_b;
    
    // -- Moments -- (Body Frame)
    Eigen::Vector3d M_scaled = Eigen::Vector3d(C_aero.C_moments(0) * wingspan, // Roll
                                                C_aero.C_moments(1) * MAC_chord_length, // Pitch
                                                C_aero.C_moments(2) * wingspan); // Yaw
    Eigen::Vector3d M_aero_b = dyn_pressure * ref_area * M_scaled;
    Eigen::Vector3d M_b = M_aero_b; // Simple, assumed thrust acting through the COG.
    
    // ---- dXdt.omega ---- (body)
    Eigen::Vector3d w = X.omega;
    Eigen::Vector3d ang_momentum = J * w; // Gyroscopic  term
    dXdt.omega = J.llt().solve(M_b - w.cross(ang_momentum));
    return dXdt;
}

PNresult AMRAAM::pro_nav_2d(const double N, const State& X_missile, const State& X_targ, double collision_radius) const{
    /*
    Pro-nav in (x,y) with z-axis rotation.
    */
   PNresult out;
   const Eigen::Vector2d r{
       X_targ.pos(0) - X_missile.pos(0),
        X_targ.pos(1) - X_missile.pos(1) 
    };
    out.collided = (std::hypot(r(0), r(1)) <= collision_radius) ? true : false;
    Eigen::Vector3d v_targ_i   = X_targ.q   * X_targ.vel;
    Eigen::Vector3d v_missile_i = X_missile.q * X_missile.vel;
    const Eigen::Vector2d r_dot{
        v_targ_i(0) - v_missile_i(0),
        v_targ_i(1) - v_missile_i(1)
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