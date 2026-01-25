#pragma once
#include <stdio.h>
#include "Common.hpp"

struct monte_carlo_params{
    int i = 0;
    double dt = 0.1; //s
    double T = 0.0; //s
};

struct quaternion{
    double q1;
    double q2;
    double q3;
    double q4;
};

template<typename State, typename deriv>
State rk4_step(State X, monte_carlo_params& mc, deriv&& dXdt){ // passes in states of various size and computes numerical integration
    // eventually pass in a function when the functions differ
        auto k1 = dXdt(mc.T, X);
        auto k2 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k1);
        auto k3 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k2);
        auto k4 = dXdt(mc.T + mc.dt, X + mc.dt*k3);
        X = X + (mc.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        mc.T += mc.dt;
    return X;
}  

quaternion euler_to_quat(double phi, double theta, double psi){
    // const Eigen::Matrix4d;
    quaternion q;
    return q;
}