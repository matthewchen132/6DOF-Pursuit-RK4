#include <Eigen/Dense>

class pitch_eq{
    /*
    Uses Newton Raphson to find the pitching trim state (alpha, V, and elevator angle) at which moments are in balance.
    
    Feed this into the PI controller such that:
     -   e = q_cmd - q_current
     -   elevator_angle = elevator_angle_trim(alpha) + PI(q_cmd - q)
    
    */
   struct pitch_trim{
       // Trim state which we want to get to for equilibrium.
       double alpha = 0.0;
       double V_setpoint= 0.0;
       double elevator_angle = 0.0;
   };
   pitch_trim find_pitch_trim(const double V_setpoint, const double alpha);
       // Cm = Cm_0 + (Cm_alpha*alpha) + Cm_q * q_hat + + Cm_e * elevator_angle; // pitch moment (C_m)


};