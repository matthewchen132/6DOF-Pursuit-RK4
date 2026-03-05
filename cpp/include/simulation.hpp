#pragma once
#include <stdio.h>
#include "Common.hpp"
#include <initializer_list>
#include <matplot/matplot.h>
#include <Eigen/Geometry>

struct monte_carlo_params{
    int i = 0;
    double dt = 0.025; //s
    double T = 0.0; //s
};


template<typename State, typename deriv>
State rk4_step(State X, monte_carlo_params& mc, deriv&& dXdt){ // passes in states of various size and computes numerical integration
    // eventually pass in a function when the functions differ
    auto k1 = dXdt(mc.T, X);
    auto k2 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k1);
    auto k3 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k2);
    auto k4 = dXdt(mc.T + mc.dt, X + mc.dt*k3);
    X = X + (mc.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    return X;
}  

Eigen::Quaterniond velocities_to_quat(double u, double v, double w){
    /*
    u,v,w is velocities in r,p,y directions in the body frame.
    takes current v
    */
    Eigen::Quaternion q = Eigen::Quaterniond::Identity();
        Eigen::AngleAxisd();

    return q;
}

