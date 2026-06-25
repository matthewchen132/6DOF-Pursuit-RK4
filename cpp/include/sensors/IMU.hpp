#pragma once
#include <random>
#include <cmath>
#include <Eigen/Dense>
#include "objects/SimConfig.hpp"

/*
IMU error model. Each output channel = truth + 3 stacked error processes:

  1. White noise (ARW/VRW)        : std_discrete = density / sqrt(dt)
  2. Rate Random Walk  (Brown)    : b += K * sqrt(dt) * N(0,1)        -> Var grows linearly, unbounded
  3. Bias Instability  (1st-order : b  = exp(-dt/tau)*b + N(0, sigma_BI*sqrt(1-exp(-2dt/tau)))
     Gauss-Markov, pink-ish)         -> Var saturates at sigma_BI^2 (the bias-instability floor)

All dt-dependent factors are baked into the distributions at construction (dt is fixed).
Magnitudes below should be read off the target IMU's datasheet / Allan-deviation plot:
   -1/2 slope -> ARW/VRW,   flat floor -> bias instability,   +1/2 slope -> rate random walk.
*/
class IMU {
    public:
        Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_b = Eigen::Vector3d::Zero();
        bool dropped_out = false;

        IMU(double dt, const SimConfig& cfg = SimConfig())
        :
        dt_(dt),
        rng(make_rng(cfg)),

        //  -- White Noise --
        gyro_white_dist (0.0, ARW / std::sqrt(dt)),
        accel_white_dist(0.0, VRW / std::sqrt(dt)),

        // -- Rate random walk (Brownian motion): per-step increment std = K * sqrt(dt) --
        gyro_rrw_dist (0.0, GYRO_RRW_K  * std::sqrt(dt)),
        accel_rrw_dist(0.0, ACCEL_RRW_K * std::sqrt(dt)),
        
        // Gauss-Markov driving noise: std = sigma_BI * sqrt(1 - exp(-2 dt / tau))
        gyro_gm_dist (0.0, GYRO_BI_SIGMA  * std::sqrt(1.0 - std::exp(-2.0 * dt / GYRO_BI_TAU))),
        accel_gm_dist(0.0, ACCEL_BI_SIGMA * std::sqrt(1.0 - std::exp(-2.0 * dt / ACCEL_BI_TAU)))
        {
            // Initialize the Gauss-Markov bias at its stationary distribution N(0, sigma_BI),
            // so each Monte-Carlo run starts with a different (but spec-consistent) turn-on bias.
            std::normal_distribution<double> gyro_bi_init (0.0, GYRO_BI_SIGMA);
            std::normal_distribution<double> accel_bi_init(0.0, ACCEL_BI_SIGMA);
            gyro_gm_  = noise3(gyro_bi_init);
            accel_gm_ = noise3(accel_bi_init);
        }

        void update(const Eigen::Vector3d& true_omega, const Eigen::Vector3d& true_accel) {
            propagate_bias();  // updates brown + pink bias states one step
            omega_b = true_omega + gyro_rrw_  + gyro_gm_  + noise3(gyro_white_dist);
            accel_b = true_accel + accel_rrw_ + accel_gm_ + noise3(accel_white_dist);
            // TODO: g_sensitivity (gyro corrupted by linear accel)
        }

    private:
        double dt_;

        // NOTE: These value of ARW, VRW, rate random walk, and gauss-markov are vibed out.
        // (ARW is a real 0.01 deg/sqrt(hr) conversion; the rest are NOT device values.)

        // White noise  (Allan -1/2 slope) | Contributes short term random noise
        static constexpr double ARW = 0.01 * M_PI / 10800.0; // 0.01 deg/sqrt(hr) -> rad/sqrt(s)   [OK]
        static constexpr double VRW = 0.01;                  // (m/s)/sqrt(s)   <-- CONFIRM UNITS vs datasheet

        // Rate Random Walk | Long-term brown noise
        static constexpr double GYRO_RRW_K  = 1.0e-5;        // (rad/s)/sqrt(s)   [Replace value with IMU specs]
        static constexpr double ACCEL_RRW_K = 1.0e-3;        // (m/s^2)/sqrt(s)   [Replace value with IMU specs]

        // Bias instability floor (Allan flat minimum) | Medium term noise
        static constexpr double GYRO_BI_SIGMA  = 5.0e-5;     // rad/s   steady-state std   
        static constexpr double GYRO_BI_TAU    = 100.0;      // s       correlation time   [Replace value with IMU specs]
        static constexpr double ACCEL_BI_SIGMA = 1.0e-2;     // m/s^2                      [Replace value with IMU specs]
        static constexpr double ACCEL_BI_TAU   = 100.0;      // s                          [Replace value with IMU specs]

        // Bias states (carried across timesteps)
        Eigen::Vector3d gyro_rrw_  = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_rrw_ = Eigen::Vector3d::Zero();
        // - Gauss Markov to model pink noise. Has one timestep of memory
        Eigen::Vector3d gyro_gm_   = Eigen::Vector3d::Zero();
        Eigen::Vector3d accel_gm_  = Eigen::Vector3d::Zero();

        std::mt19937 rng;
        std::normal_distribution<double> gyro_white_dist;
        std::normal_distribution<double> accel_white_dist;
        std::normal_distribution<double> gyro_rrw_dist;
        std::normal_distribution<double> accel_rrw_dist;
        std::normal_distribution<double> gyro_gm_dist;
        std::normal_distribution<double> accel_gm_dist;

        void propagate_bias() {
            // -- Brownian motion / Bias Rate Random Walk --
            gyro_rrw_  += noise3(gyro_rrw_dist); // bias (rad/s)
            accel_rrw_ += noise3(accel_rrw_dist); // bias (m/s^2)

            // -- Pink Noise Modeling ; 1st-order Gauss-Markov (1/f correlation) -- 
            const double a_g = std::exp(-dt_ / GYRO_BI_TAU);
            const double a_a = std::exp(-dt_ / ACCEL_BI_TAU);
            gyro_gm_  = a_g * gyro_gm_  + noise3(gyro_gm_dist);
            accel_gm_ = a_a * accel_gm_ + noise3(accel_gm_dist);
        }

        Eigen::Vector3d noise3(std::normal_distribution<double>& dist) {
            return {dist(rng), dist(rng), dist(rng)};
        }
};
