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
    double dt = 0.02; //s 0.2s and try
    double T = 0.0; //s
};

// 639.994    -511.578     -297.17     642.001    -513.016    -301.245    -7.76242    0.235373    -13.6828     4.76491
template<typename S, typename deriv>
inline S rk4_step(S X, monte_carlo_params& mc, deriv&& f){ // passes in states of various size and computes numerical integration
    // TODO: Lots of temp variables
    S k1 = f(mc.T, X);
    S k2 = f(mc.T + .5*mc.dt, X + .5*mc.dt*k1);
    S k3 = f(mc.T + .5*mc.dt, X + .5*mc.dt*k2);
    S k4 = f(mc.T + mc.dt, X + mc.dt*k3);
    X = X + (mc.dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    return X;
}  

inline std::vector<double> mat_to_vec(const Eigen::MatrixXd& M, int column, size_t filled_rows, bool flip_sign){
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

inline void generate_mp4(const std::filesystem::path& output_dir, matplot::axes_handle ax, matplot::figure_handle fig){
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

inline void equalize_axes(double size, matplot::axes_handle ax){
    ax->xlim({-size, size});
    ax->ylim({-size, size});
    ax->zlim({-size, size});
}

// Note to self: inline lets you have multiple defs
inline Eigen::Vector3d actuator_limit(Eigen::Vector3d& cmd, double limit){
    if (cmd.norm() > limit){
        cmd = cmd.normalized() * limit;
    }
    return cmd;
}
