#include "simulation.hpp"
#include "AMRAAM.hpp"
#include "evader.hpp"
#include <stdio.h>
#include <iostream>
#include "Common.hpp"
#include <matplot/matplot.h>


int main(){
    monte_carlo_params run{};
    Evader target{};

    std::vector<double> t_log, x_log, y_log;

    while(run.T <= 5.0){
        t_log.push_back(run.T);
        x_log.push_back(target.X_2d(0));
        y_log.push_back(target.X_2d(1));
        target.X_2d = rk4_step(
            target.X_2d, 
            run, 
            [&](double t, const Evader::State6& X) { 
                return target.dXdt_2d(run.T, X); 
            }
        );
    }

    matplot::plot(x_log, y_log, "r*");
    matplot::xlabel("X position (m)");
    matplot::ylabel("Y position (m)");
    matplot::title("X and Y position");
    matplot::show();

    std::cout << "End of Simulation" << std::endl;
    return 0;
}