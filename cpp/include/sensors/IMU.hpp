#pragma once
#include <random>
#include <Eigen/Dense>
#include "objects/SimConfig.hpp"

class IMU {
public:
    Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_b = Eigen::Vector3d::Zero();
    bool dropped_out = false;

    IMU(double dt, const SimConfig& cfg = SimConfig())
    :
    rng(make_rng(cfg)),
    gyro_noise_dist(0.0, ARW / sqrt(dt)),
    accel_noise_dist(0.0, VRW / sqrt(dt))
    {}

    void update(const Eigen::Vector3d& true_omega, const Eigen::Vector3d& true_accel) {
        omega_b = true_omega + noise3(gyro_noise_dist);
        accel_b = true_accel + noise3(accel_noise_dist);
        // TODO: add_brown_noise (Rate Random Walk — bias drift)
        // TODO: add_bias_instability (Pink Noise — Gauss-Markov bias)
        // TODO: g_sensitivity (gyro corrupted by linear accel)
    }

private:
    static constexpr double ARW = 0.01 * M_PI / 10800; // rads/sqrt(s)
    static constexpr double VRW = 0.05;                 // (m/s)/sqrt(s)

    std::mt19937 rng;
    std::normal_distribution<double> gyro_noise_dist;
    std::normal_distribution<double> accel_noise_dist;

    Eigen::Vector3d noise3(std::normal_distribution<double>& dist) {
        return {dist(rng), dist(rng), dist(rng)};
    }
};
