#include "simulation.hpp"
#include "flight_object_headers/AMRAAM.hpp"
#include "flight_object_headers/evader.hpp"
#include "flight_object_headers/wind_vector.hpp"
#include "state_full.hpp"
#include "performance_tools/Timer.hpp"
#include "aero/aero.hpp"
#include <stdio.h>
#include <iostream>
#include <numeric>
#include <matplot/matplot.h>
#include <math.h>
#include <thread>


struct log_row{ // option for full logging
    double t;
    Eigen::Vector3d missile_pos;
    Eigen::Vector3d target_pos;
};

int main(){
    aero_functions aero_functions; // Aerodynamic helpers
    const double blast_radius = 2.0; //m
    const int sim_time = 20;
    monte_carlo_params mc{};
    Timer loop_timer_us;
    Timer rk4_timer_us;
    // -- Missile (world frame)--
    AMRAAM missile{};
    missile.X.pos = {0.0, 0.0, 0.0};
    missile.X.vel = {30.0, 0.0, 0.0};
    missile.X.q = {1.0,0,0,0};
    AeroAngles missile_aero_angles;
    // -- Target --
    Evader target{};
    target.X.pos = {200.0, 0.0, 0.0};
    target.X.vel = {1.0, 1.0, 1.0};
    target.X.q = {1.0,0,0,0};
    AeroAngles target_aero_angles;

    // -- Wind --
    wind_gust gust;
    gust.t_start = 0.0;
    gust.t_end = sim_time;
    gust.wind_vel = Eigen::Vector3d(10.0, 0.0, 0.0);
    target.wind_vel_i = gust.wind_vel; // pass to object to calculate aerodynamic angles
    missile.wind_vel_i = gust.wind_vel; // pass to object to calculate aerodynamic angles
    
    bool missile_collided = false;
    size_t counter = 0;
    std::vector<double> loop_times;
    loop_times.reserve(int(sim_time/mc.dt));
    
    Eigen::MatrixXd pos_log(int(sim_time/mc.dt)+1, 11);
    while(mc.T <= sim_time && !missile_collided){
        // -- Timer --
        loop_timer_us.start_timing();
        
        // -- position logging -- 
        Eigen::Vector3d a_n = missile.pro_nav_6dof(3.0, missile.X, target.X);
        pos_log(counter,0) = mc.T;
        pos_log.block<1,3>(counter,1) = missile.X.pos.transpose();
        pos_log.block<1,3>(counter,4) = target.X.pos.transpose();
        pos_log.block<1,3>(counter,7) = a_n.transpose();
        pos_log(counter, pos_log.cols()-1) = (missile.X.pos - target.X.pos).norm(); // ZEM

        // AeroAngles target_wind_angles = aero_functions.recalc_aero_angles(target.X, gust.wind_vel); // angle of attack & sideslip  
        target.X = rk4_step(
            target.X, mc, 
            [&](double t, const State& X) { 
                return target.dXdt(t, X); 
            });
        target.X.q.normalize();

        // -- calculate angle of attack, sideslip -- 
        // AeroAngles missile_wind_angles = aero_functions.recalc_aero_angles(missile.X, gust.wind_vel); // angle of attack & sideslip 
        missile.X = rk4_step(
            missile.X, mc,
            [&](double t, const State& X) { 
                return missile.dXdt(t, X, a_n); 
            });
        missile.X.q.normalize();

        // Missile Collision detecion
        if ((missile.X.pos - target.X.pos).norm() <= blast_radius ) {
            std::cout << "Collision occured at time: " << mc.T << std::endl;
            missile_collided = true;
        }        
        
        // -- Log Loop Time --
        double sim_loop_time = loop_timer_us.stop_timing();
        loop_times.push_back(sim_loop_time); 
        
        // -- Increment --
        counter++;
        mc.T += mc.dt;
    }
    double total_sim_time = std::accumulate(loop_times.begin(), loop_times.end(), 0.0);
    double average_sim_time = total_sim_time / counter;
    std::cout << "Average Sim Loop Time (us): " << average_sim_time << std::endl;

    // -- Plotting --
    pos_log.conservativeResize(counter, Eigen::NoChange); // (WIP) find smart way to not hardcode "11"
    std::vector<double> m_x = mat_to_vec(pos_log, 1, counter, false);
    std::vector<double> m_y = mat_to_vec(pos_log, 2, counter, false);
    std::vector<double> m_z = mat_to_vec(pos_log, 3, counter, true);
    std::vector<double> t_x = mat_to_vec(pos_log, 4, counter, false);
    std::vector<double> t_y = mat_to_vec(pos_log, 5, counter, false);
    std::vector<double> t_z = mat_to_vec(pos_log, 6, counter, true);
    
    auto fig = matplot::figure(true);
    auto ax = fig->current_axes();
    ax->hold(matplot::on);

    // -- F-16 TRAJECTORY --
    auto targ_plot = ax->plot3(t_x, t_y, t_z);
    targ_plot->color("blue");
    targ_plot->line_width(1.5);
    targ_plot->display_name("Target Location");
    // -- mark start point as "o" --
    auto targ_start = ax->plot3({t_x[0]}, {t_y[0]}, {t_z[0]}, "bo");
    targ_start->marker_size(10);
    targ_start->display_name("Target Start");
    // -- mark end point as "o" --    
    auto targ_end = ax->plot3({t_x[t_x.size()-1]}, {t_y[t_x.size()-1]}, {t_z[t_x.size()-1]}, "bx");
    targ_end->marker_size(10);
    targ_end->display_name("Target End");
    // -- MISSILE TRAJECTORY --
    auto m_plot = ax->plot3(m_x, m_y, m_z);
    m_plot->color("red");
    m_plot->line_width(1.5);
    m_plot->display_name("Missile Location");
    // -- mark start point as "o" --
    auto m_start = ax->plot3({m_x[0]}, {m_y[0]}, {m_z[0]}, "ro");
    m_start->marker_size(10);
    m_start->display_name("Missile Start");
    // -- mark end point as "x" --
    auto m_end = ax->plot3({m_x[m_x.size()-1]}, {m_y[m_x.size()-1]}, {m_z[m_x.size()-1]}, "rx");
    m_end->marker_size(10);
    m_end->display_name("Missile End");
    // -- Label Axes --
    ax->xlabel("X position (m)");
    ax->ylabel("Y position (m)");
    ax->zlabel("Z position (m)");
    ax->title("Interceptor vs F-16");
    matplot::legend();

    // -- Generate a Video of Trajectory (Replace path with your own)--
    char video_response;
    std::cout << "Generate video and ZEM log of sim? [y]/[n]:  ";
    std::cin >> video_response;
    if(video_response == 'y' || video_response == 'Y'){
        // -- Edit path to your choice -- 
        generate_mp4("/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/sim_visuals", ax, fig); //generates a revolving mp4 of the sim for visibility / debugging
        // (WIP) generate_logs_csv("~/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/sim_visuals", pos_log);
    }    

    std::cout << "End of Simulation" << std::endl;
    std::cout << pos_log << std::endl;
    std::cout << "   missile_x   " << "  missile_y   " << "  missile_z   " << "   target_x   " << "   target_y   " << "   target_z   " << "           a_x   " << "           a_y   " << "           a_z   " << "    ZEM_dist   " << std::endl;

    return 0;
}