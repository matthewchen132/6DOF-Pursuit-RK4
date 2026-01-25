#include <Eigen/Dense>
#include <math.h>
#include <algorithm>
#include <string>
#include "evader.hpp"
#include "Common.hpp"


Evader::State6 Evader::dXdt_2d(double T, const State6& X_2d) const{ // const function means it wont modify the variables
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
        return dXdt_2d;
    }