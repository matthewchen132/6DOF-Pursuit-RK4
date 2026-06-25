#pragma once
#include <Eigen/Dense>
#include <random>
#include "objects/SimConfig.hpp"
#include "objects/State.hpp"
#include "sensors/IMU.hpp"

/*
Produces a noisy State from simulated IMU + GPS readings.
Call update_noisy_state() each timestep; the returned State is 
substituted directly into Evader::f() in place of the true State.

[X] Angular / Velocity Random Walk (White Noise) via IMU class
[X] GPS Simple White Noise on pos_i
[X] Brown Noise / Rate Random Walk (IMU bias drift)
[ ] Pink Noise / Bias Instability (Gauss-Markov)
[ ] Realistic sensor poll rates + dead reckoning between GPS updates
[ ] Kalman filter correction step
*/

class SensorState {
    /*
    Does NOT account for: Scale Factor, Non-orthogonality, Nonlinearity,
    Turn-on DC bias, g^2 sensitivity.
    */
    public:
        SensorState(double dt, const SimConfig& cfg = SimConfig())
        :
        imu_(dt, cfg),
        gps_noise_dist(0.0, 0.5),
        rng(make_rng(cfg)),
        dt_(dt)
        {}

    // true_accel: specific force from dynamics (f.vel_b from Evader::f)
    State update_noisy_state(const State& X_true, const Eigen::Vector3d& true_accel) {
        imu_.update(X_true.omega_b, true_accel);

        // GPS: corrupt inertial position with white noise
        X_noisy_.pos_i = X_true.pos_i + noise3(gps_noise_dist);

        // Velocity: integrate noisy specific force
        X_noisy_.vel_b += imu_.accel_b * dt_;

        // Angular velocity
        X_noisy_.omega_b = imu_.omega_b;

        // Quaternion propagation: dq/dt = 0.5 * q_bi * omega_q
        Eigen::Quaterniond omega_q(0.0, imu_.omega_b(0), imu_.omega_b(1), imu_.omega_b(2));
        Eigen::Quaterniond dqdt = X_noisy_.q_bi * omega_q;
        dqdt.coeffs() *= 0.5;
        X_noisy_.q_bi.coeffs() += dqdt.coeffs() * dt_;
        X_noisy_.q_bi.normalize();
        X_noisy_.q_ib = X_noisy_.q_bi.conjugate();
        return X_noisy_;
    }

private:
    double dt_;
    IMU imu_;
    State X_noisy_;
    std::normal_distribution<double> gps_noise_dist;
    std::mt19937 rng;
    Eigen::Vector3d noise3(std::normal_distribution<double>& dist) {
        return {dist(rng), dist(rng), dist(rng)};
    }
};
