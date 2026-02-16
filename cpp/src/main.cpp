#include "simulation.hpp"
#include "flight_object_headers/AMRAAM.hpp"
#include "flight_object_headers/evader.hpp"
#include "Common.hpp"
#include "flight_object_headers/wind_vector.hpp"
#include <stdio.h>
#include <iostream>
#include <matplot/matplot.h>
#include "state_full.hpp"

int main(){
    double collision_radius = 2.0; //m
    monte_carlo_params mc{};
    Evader target1{};
    wind wind{}; 
    AMRAAM missile{};
    // missile1.X = {100,0,30,10,0,0};
    // target1.X = {50,70,10,10,10,10};
    missile.X.pos = {0,0,0};
    missile.X.vel = {30,10,0};
    target1.X.pos = {50,70,0};
    target1.X.vel = {10,0,0};


    bool missile_collided = false;
    std::vector<double> t_log1, x_log1, y_log1, 
                        x_log_missile, y_log_missile, 
                        a_norm, dLOS_dt, v_closing;

    while(mc.T <= 30.0){
        t_log1.push_back(mc.T);
        x_log1.push_back(target1.X(0));
        y_log1.push_back(target1.X(1));
        x_log_missile.push_back(missile.X(0));
        y_log_missile.push_back(missile.X(1));
        const PNresult PN = missile.pro_nav_2d(5.0, missile.X, target1.X, collision_radius);

        target1.X = rk4_step(
            target1.X, mc, 
            [&](double t, const Evader::State& X) { 
                return target1.dXdt_2d(t, X); 
            });

        missile.X = rk4_step(
            missile1.X, mc,
            [&](double t, const AMRAAM::State& X) { 
                return missile1.dXdt_2d(t, X, PN.a_norm); 
            });

        a_norm.push_back(PN.a_norm);
        dLOS_dt.push_back(PN.dLOS_dt);
        v_closing.push_back(PN.v_closing);
        missile_collided = PN.collided;

        if (missile_collided) { 
            std::cout << "Collision occured at time" << t_log1.back() << std::endl;
            break;
        }

        mc.T += mc.dt;
    }
    //Plotting
    matplot::figure();
    auto targ_plot = matplot::plot(x_log1, y_log1, "r-");
    targ_plot->display_name("Target Location");
    matplot::hold(matplot::on);
    auto m_plot = matplot::plot(x_log_missile, y_log_missile, "b-");
    m_plot->display_name("Missile Location");
    matplot::xlabel("X position (m)");
    matplot::ylabel("Y position (m)");
    matplot::title("X and Y position");
    matplot::legend();
    matplot::show();

    std::cout << "End of Simulation" << std::endl;
    return 0;
}