#pragma once
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <stdexcept>
#include <Eigen/Dense>

class SimLogger {
public:
    explicit SimLogger(const std::filesystem::path& path) : file_(path) {
        if (!file_.is_open()) {
            throw std::runtime_error("SimLogger: could not open file: " + path.string());
        }
        file_ << "time,roll,pitch,yaw\n";
    }

    void log_rpy(double time, const Eigen::Vector3d& rpy) {
        file_ << std::fixed << std::setprecision(6)
              << time   << ','
              << rpy(0) << ','
              << rpy(1) << ','
              << rpy(2) << '\n';
    }

private:
    std::ofstream file_;
};
