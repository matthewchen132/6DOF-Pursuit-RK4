// -- Simulation tools -- 
#include "simulation.hpp"
#include "quat_rpy/quaternion_utils.hpp"
#include "sim_performance_tools/Timer.hpp"
#include "objects/SimConfig.hpp"

// -- Objects --
#include "flight_object_headers/AMRAAM.hpp"
#include "flight_object_headers/evader.hpp"
#include "flight_object_headers/wind_vector.hpp"
#include "aero/aero.hpp" // Aero
#include "plotting/printing.hpp" // Plotting
#include "objects/SensedState.hpp"

// -- libraries --
#include <stdio.h>
#include <iostream>
#include <numeric>
#include <matplot/matplot.h>
#include <cmath>
#include <thread>

struct log_row{ // option for full logging
    double t;
    Eigen::Vector3d missile_pos;
    Eigen::Vector3d target_pos;
};

int main(){
    constexpr double sim_end_time = 30.0;
    monte_carlo_params mc{};
    Timer loop_timer_us;
    Timer rk4_timer_us;
    SimConfig cfg;

    // -- Missile (world frame)--
    Eigen::Vector3d missile_vel_i = {20.0, 0.0, 0.0}; 
    AMRAAM missile(1000, 17000, 0.45, 2.0, 10.0, missile_vel_i);
    missile.X.pos_i = {0.0, 0.0, 0.0};

    // -- Target (F-16 Model) --
    Eigen::Vector3d f16_vel_i = {0.0, 10.0, 10.0};
    Evader f_16(9298.64, 120000, 1.0, 3.2, 10.0, f16_vel_i);
    SensorState sensed = SensorState(mc.dt, cfg);
    f_16.X.pos_i = {600.0, 100.0, -300.0};

    // -- Wind --
    wind_gust gust;
    gust.t_start = 0.0;
    gust.t_end = sim_end_time;
    gust.wind_vel = Eigen::Vector3d(0.0, 0.0, 0.0); // Passes A unit test where 0 wind = same position as without wind incorporated
    
    bool missile_collided = false;
    size_t counter = 0;
    std::vector<double> loop_times;
    loop_times.reserve(int(sim_end_time/mc.dt));
    Eigen::MatrixXd pos_log(int(sim_end_time/mc.dt)+1, 14);
    Eigen::MatrixXd rpy_log(int(sim_end_time/mc.dt)+1, 7); //1 time 3 missile rpy, 3 target rpy
    Eigen::MatrixXd aero_log(int(sim_end_time/mc.dt)+1, 5); // 1 time, angle of attack, sideslip, 3 Coeffs of drag lift sideforce for both missile and target

    while(mc.T <= sim_end_time && !missile_collided){
        // -- Timer --
        loop_timer_us.start_timing();
        
        // -- Posittion Data Logging -- 
        Eigen::Vector3d a_n_i = missile.pro_nav_6dof(3.0, missile.X, f_16.X);
        pos_log(counter,0) = mc.T;

        // -- Positions (Inertial) --
        pos_log.block<1,3>(counter,1) = missile.X.pos_i.transpose();
        pos_log.block<1,3>(counter,4) = f_16.X.pos_i.transpose();

        // -- A_norm / ZEM  --
        pos_log.block<1,3>(counter,7) =  a_n_i.transpose();
        pos_log.block<1,3>(counter,10) = missile.X.pos_i - f_16.X.pos_i; // ZEM
        pos_log(counter, pos_log.cols()-1) = (missile.X.pos_i - f_16.X.pos_i).norm(); // ZEM

        // -- Object Roll Pitch Yaw Logging --
        rpy_log(counter,0) = mc.T;
        rpy_log.block<1,3>(counter,1) = (quaternion_to_euler_ZYX(missile.X.q_bi)*180/M_PI).transpose();
        rpy_log.block<1,3>(counter,4) = (quaternion_to_euler_ZYX(f_16.X.q_bi)*180/M_PI).transpose();

        // -- Aerodynamic Logging -- 
        AeroAngles m_aero = missile.get_aero_angles(missile.X, gust.wind_vel);
        AeroAngles f16_aero = f_16.get_aero_angles(f_16.X, gust.wind_vel);
        aero_log(counter, 0) = mc.T;
        aero_log(counter, 1) = m_aero.alpha * 180.0 / M_PI;
        aero_log(counter, 2) = m_aero.beta  * 180.0 / M_PI;
        aero_log(counter, 3) = f16_aero.alpha * 180.0 / M_PI;
        aero_log(counter, 4) = f16_aero.beta  * 180.0 / M_PI;
        
        f_16.X = rk4_step(f_16.X, mc,
            [&](const State& X) {
                return f_16.f(X, gust.wind_vel, mc.dt);
            });
        f_16.X.q_ib.normalize();
        f_16.X.q_bi.normalize();
        f_16.update_actuator(mc.dt);

        missile.X = rk4_step(missile.X, mc,
            [&](const State& X) {
                return missile.f(X, a_n_i, gust.wind_vel);
            });
        missile.X.q_ib.normalize();
        missile.X.q_bi.normalize();

        missile_collided = collision_detected(missile.X.pos_i, f_16.X.pos_i, missile.blast_radius, mc.T); // Collision Detection
        double sim_loop_time = loop_timer_us.stop_timing(); // Log timer        
        // -- Increment --
        loop_times.push_back(sim_loop_time); 
        counter++;
        mc.T += mc.dt;
    }
    double total_sim_time = std::accumulate(loop_times.begin(), loop_times.end(), 0.0);
    double average_sim_time = total_sim_time / counter;
    std::cout << "Average Sim Loop Time (us): " << average_sim_time << std::endl;

    // -- Plotting --
    pos_log.conservativeResize(counter, Eigen::NoChange);
    rpy_log.conservativeResize(counter, Eigen::NoChange);  // add this
    aero_log.conservativeResize(counter, Eigen::NoChange); // and this    
    std::vector<double> t_vec = mat_to_vec(pos_log, 0, counter, false);
    std::vector<double> m_x = mat_to_vec(pos_log, 1, counter, false);
    std::vector<double> m_y = mat_to_vec(pos_log, 2, counter, false);
    std::vector<double> m_z = mat_to_vec(pos_log, 3, counter, true);
    std::vector<double> t_x = mat_to_vec(pos_log, 4, counter, false);
    std::vector<double> t_y = mat_to_vec(pos_log, 5, counter, false);
    std::vector<double> t_z = mat_to_vec(pos_log, 6, counter, true);
    std::vector<double> a_x = mat_to_vec(pos_log, 7, counter, false);
    std::vector<double> a_y = mat_to_vec(pos_log, 8, counter, false);
    std::vector<double> a_z = mat_to_vec(pos_log, 9, counter, false);

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
    // -- Wind Velocity Direction --

    double scale = 100.0; // just for visualization
    wind w(ax);
    w.plot_wind_dir(gust, scale);

    // -- Label Axes --
    ax->xlabel("X position (m)");
    ax->ylabel("Y position (m)");
    ax->zlabel("Z position (m)");
    ax->title("Interceptor vs F-16");
    matplot::legend();
    
    // -- Generate a Video of Trajectory (Replace path with your own)--
    char video_response;
    std::cout << "Generate video and ZEM log of sim? [y/n]:  ";
    std::cin >> video_response;
    if(video_response == 'y' || video_response == 'Y'){
        // -- Edit path to your choice -- 
        generate_mp4("/Users/awolf/Code/6DOF-Missile-Sim/6DOF-Pursuit-RK4/cpp/sim_visuals", ax, fig); //generates a revolving mp4 of the sim for visibility / debugging
    }    

    display_positions(pos_log);    
    display_rpy(rpy_log);
    display_aero(aero_log);

    return 0;
}