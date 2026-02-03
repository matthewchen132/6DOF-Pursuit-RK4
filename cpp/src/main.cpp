#include "simulation.hpp"
#include "AMRAAM.hpp"
#include "evader.hpp"
#include <stdio.h>
#include <iostream>
#include "Common.hpp"
#include <matplot/matplot.h>

void plot_everything(auto x_log1, auto y_log1, auto x_log_missile, auto y_log_missile, auto t_log1, auto a_norm, auto dLOS_dt, auto v_closing){
    //make a matrix later
    matplot::subplot(2,2,1);
    matplot::plot(x_log1, y_log1, "r*");
    matplot::hold(matplot::on);
    matplot::plot(x_log_missile, y_log_missile, "b*");
    matplot::xlabel("X position (m)");
    matplot::ylabel("Y position (m)");
    matplot::title("X and Y position");

    matplot::subplot(2,2,2);
    matplot::plot(t_log1, a_norm);
    matplot::xlabel("Time (s)");
    matplot::ylabel("Anorm (m/s^2)");
    matplot::title("Anorm (pronav)");    

    matplot::subplot(2,2,3);
    matplot::plot(t_log1, dLOS_dt);
    matplot::xlabel("Time (s)");
    matplot::ylabel("dLOSdt (rads/s)");
    matplot::title("LOS w.r.t time (pronav)");    

    matplot::subplot(2,2,4);
    matplot::plot(t_log1, v_closing);
    matplot::xlabel("Time (s)");
    matplot::ylabel("vclosing (m/s^2)");
    matplot::title("vclosing (pronav)");    
    matplot::show();
}

int main(){
    monte_carlo_params mc{};
    Evader target1{};
    target1.X_2d = {100,100,10,10,10,10};
    AMRAAM missile1{};
    missile1.X_2d = {0,0,30,0,0,0};

    bool missile_collided = false;
    std::vector<double> t_log1, x_log1, y_log1, 
                        x_log_missile, y_log_missile, 
                        a_norm, dLOS_dt, v_closing;

    while(mc.T <= 30.0){
        t_log1.push_back(mc.T);
        x_log1.push_back(target1.X_2d(0));
        y_log1.push_back(target1.X_2d(1));
        x_log_missile.push_back(missile1.X_2d(0));
        y_log_missile.push_back(missile1.X_2d(1));
        target1.X_2d = rk4_step(
            target1.X_2d, mc, 
            [&](double t, const Evader::State6& X) { 
                return target1.dXdt_2d(t, X); 
            });
        missile1.X_2d = rk4_step(
            missile1.X_2d, mc, 
            [&](double t, const AMRAAM::State6& X) { 
                const PNresult PN = missile1.pro_nav_2d(5.0, missile1.X_2d, target1.X_2d);
                a_norm.push_back(PN.a_norm);
                dLOS_dt.push_back(PN.dLOS_dt);
                v_closing.push_back(PN.v_closing);
                missile_collided = PN.collided;
                return missile1.dXdt_2d(t, X, PN.a_norm); 
            });
        std::cout << "Evader (u,v) " << target1.X_2d(2) << " , " << target1.X_2d(3) << std::endl;    
        std::cout << "Missile (u,v) " << missile1.X_2d(2) << " , " << missile1.X_2d(3) << std::endl;
        if (missile_collided) { // not working
            std::cout << "Collision occured at time" << t_log1.back() << std::endl;
            break;
        }
    }

    // plot_everything(x_log1,y_log1, x_log_missile, y_log_missile, t_log1, a_norm, dLOS_dt, v_closing);
    matplot::plot(x_log1, y_log1, "r*");
    matplot::hold(matplot::on);`
    matplot::plot(x_log_missile, y_log_missile, "b*");
    matplot::xlabel("X position (m)");
    matplot::ylabel("Y position (m)");
    matplot::title("X and Y position");
    matplot::show();

    std::cout << "End of Simulation" << std::endl;
    return 0;
}