#include "AMRAAM.hpp"


AMRAAM::State6 AMRAAM::dXdt_2d(double T, const State6& X_2d) const {
    State6 dXdt = State6::Zero();
    const double v_mag = sqrt(X_2d(2)*X_2d(2) + X_2d(3)*X_2d(3));
    const double u_portion = (v_mag > 1e-9) ? X_2d(2) : 1e-9;
    const double v_portion = (v_mag > 1e-9) ? X_2d(3) : 1e-9;
    State6 dXdt_2d = State6::Zero();
    dXdt_2d(0) = X_2d(2);
    dXdt_2d(1) = X_2d(3);
    dXdt_2d(2) = (thrust/m_evader)*cos(X_2d(4)) - (u_portion)*0.5*Cd*A*density*v_mag*v_mag/m_evader;
    dXdt_2d(3) = (thrust/m_evader)*sin(X_2d(4)) - (v_portion)*0.5*Cd*A*density*v_mag*v_mag/m_evader;
    dXdt_2d(4) = X_2d(5);
    dXdt_2d(5) = 0.0; // assumes constant headings
    return dXdt;
}

double AMRAAM::pro_nav(double Kp, const State6& X_Missile, const State6& X_Interceptor){
// Takes the r.o.c of LOS between the AMRAAM and 
// target to generate the acceleration necessary to create a collision triangle 

return a_normal;
}