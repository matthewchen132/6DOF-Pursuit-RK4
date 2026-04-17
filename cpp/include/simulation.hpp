#pragma once
#include <stdio.h>
#include <initializer_list>
#include <matplot/matplot.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <filesystem>
#include <stdio.h>
#include <vector>
#include <State.hpp>
#include <cmath>
#include "flight_object_headers/evader.hpp"

struct monte_carlo_params{
    int i = 0;
    double dt = 0.04; //s
    double T = 0.0; //s
};


template<typename S, typename deriv>
S rk4_step(S X, monte_carlo_params& mc, deriv&& dXdt){ // passes in states of various size and computes numerical integration
    // TODO: Lots of temp variables
    S k1 = dXdt(mc.T, X);
    S k2 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k1);
    S k3 = dXdt(mc.T + .5*mc.dt, X + .5*mc.dt*k2);
    S k4 = dXdt(mc.T + mc.dt, X + mc.dt*k3);
    X = X + (mc.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    return X;
}  

std::vector<double> mat_to_vec(const Eigen::MatrixXd& M, int column, size_t filled_rows, bool flip_sign){
    std::vector<double> vec(M.rows());
    if (flip_sign){
        for(int i = 0; i < filled_rows; ++i){
            vec[i] = -M(i,column);
        }
    }
    else{
        for(int i = 0; i < filled_rows; ++i){
            vec[i] = M(i,column);
        }
    }
    return vec;
}

void generate_mp4(const std::filesystem::path& output_dir, matplot::axes_handle ax, matplot::figure_handle fig){
    std::filesystem::create_directories(output_dir);
    int png_count = 0;
    for(int angle = 0; angle < 360; angle += 20){
        ax->view(angle, 35);
    fig->save((output_dir / (std::to_string(png_count) + ".png")).string());
    png_count++;
    }
    // Stitch automatically
    std::string input_pattern = (output_dir / "%d.png").string();
    std::string output_file   = (output_dir / "trajectory.mp4").string();
    std::string cmd = "ffmpeg -y -framerate 3 -i " + input_pattern
                + " -c:v libx264 -pix_fmt yuv420p " + output_file;

    if (std::system(cmd.c_str()) != 0)
        std::cerr << "ffmpeg failed\n";
}
