#include "simulation.hpp"
#include "flight_object_headers/AMRAAM.hpp"
#include "flight_object_headers/evader.hpp"
#include "Common.hpp"
#include "flight_object_headers/wind_vector.hpp"
#include <stdio.h>
#include <iostream>
#include <matplot/matplot.h>
#include "state_full.hpp"
#include <math.h>

struct log_row{ // option for full logging
    double t;
    Eigen::Vector3d missile_pos;
    Eigen::Vector3d target_pos;
};

int main(){
    double collision_radius = 2.0; //m
    monte_carlo_params mc{};
    Evader target1{};
    wind wind{}; 
    AMRAAM missile{};

    missile.X.pos = {10.0, 0.0, 0.0};
    missile.X.vel = {-10,10,10};
    missile.X.q = {1.0,0,0,0};

    target1.X.pos = {100.0, 100.0, 100.0};
    target1.X.vel = {20,20,20};
    target1.X.q = {1.0,0,0,0};

    bool missile_collided = false;
    const int sim_time = 20;
    size_t i = 0;


    Eigen::MatrixXd pos_log(int(sim_time/mc.dt)+1, 7);

    while(mc.T <= sim_time){
        // -- position logging -- 
        pos_log(i,0) = mc.T;
        pos_log.block<1,3>(i,1) = missile.X.pos.transpose();// blocks off 1 row 3 cols starting at row i, col 1
        pos_log.block<1,3>(i,4) = target1.X.pos.transpose();// blocks off 1 row 3 cols starting at row i, col 4

        Eigen::Vector3d a_n = missile.pro_nav_6dof(3.0, missile.X, target1.X);

        target1.X = rk4_step(
            target1.X, mc, 
            [&](double t, const State& X) { 
                return target1.dXdt(t, X); 
            });
        target1.X.q.normalize(); // normalize quaternions after each loop

        missile.X = rk4_step(
            missile.X, mc,
            [&](double t, const State& X) { 
                return missile.dXdt(t, X, a_n); 
            });
        missile.X.q.normalize();
        if (std::abs((missile.X.pos - target1.X.pos).norm()) <= 1.0 ) {}
            missile_collided = true;
        // if (missile_collided) { 
        //     std::cout << "Collision occured at time" << mc.T << std::endl;
        //     break;
        // }        
        i++;
        mc.T += mc.dt;
    }
    //Plotting    
    std::vector<double> m_x = mat_to_vec(pos_log, 1, i, false);
    std::vector<double> m_y = mat_to_vec(pos_log, 2, i, false);
    std::vector<double> m_z = mat_to_vec(pos_log, 3, i, true);

    std::vector<double> t_x = mat_to_vec(pos_log, 4, i, false);
    std::vector<double> t_y = mat_to_vec(pos_log, 5, i, false);
    std::vector<double> t_z = mat_to_vec(pos_log, 6, i, true);

    auto fig = matplot::figure(true);
    auto ax = fig->current_axes();

    ax->hold(matplot::on);

    auto targ_plot = ax->plot3(t_x, t_y, t_z);
    targ_plot->color("red");
    targ_plot->line_width(2.0);
    targ_plot->display_name("Target Location");

    auto m_plot = ax->plot3(m_x, m_y, m_z);
    m_plot->color("blue");
    m_plot->line_width(2.0);
    m_plot->display_name("Missile Location");

    ax->xlabel("X position (m)");
    ax->ylabel("Y position (m)");
    ax->zlabel("Z position (m)");
    ax->title("X, Y, Z position");
    matplot::legend();
    matplot::show();

    std::cout << "End of Simulation" << std::endl;
    return 0;
}