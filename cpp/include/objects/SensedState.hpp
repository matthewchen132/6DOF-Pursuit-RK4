#pragma once   
#include <Eigen/Dense>
#include <random>


// IMU_bias
// accel_bias_ += sigma_accel_walk * std::sqrt(dt) * randomVec();
// gyro_bias_  += sigma_gyro_walk  * std::sqrt(dt) * randomVec();
class SensorState {;
    /* 
        b = body frame
        i = inertial frame
        w = wind frame

        Noisy State Object
    */
    public:
        SensorState(Eigen::Vector3d accel_reading, double dt) 
        :
        accel_b(accel_reading)
        {}
        struct IMU_reading{
            Eigen::Vector3d omega_b = Eigen::Vector3d::Zero();
            Eigen::Vector3d accel_b = Eigen::Vector3d::Zero(); // 
        };
        Eigen::Vector3d pos_i = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel_b = Eigen::Vector3d::Zero();    
        Eigen::Quaterniond q_ib = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q_bi = Eigen::Quaterniond::Identity();
        double gyro_variance = 0.0;
        double accel_variance = 0.0;
        IMU_reading add_random_walk_bias(const double accel_hourly_bias, const double gyro_hourly_bias, dt){
            /*
            Adds bias per time step
             - Bias is varying, not constant. 
             - LARGEST SOURCE OF ERROR
            */ 

        }
        IMU_reading add_noise(const double accel_hourly_bias, const double gyro_hourly_bias, dt){
            // Adds white noise per time step -> integration creates a random walk
        }
        Eigen::Vector3d g_sensitivity(Eigen::Vector3d a_b){
            Linear translation add
            /*
                a_coriolis = 2w x v_center
                - With linear acceleration, we can corrupt a_coriolis, affecting our angular_velocity.
            */
        }
    private:
        std::mt19937_64 rng_;
        std::normal_distribution<double> dist_;
};

inline SensorState operator*(const State& s, double c){
    State output;
    output.pos_i = s.pos_i * c;
    output.vel_b = s.vel_b * c;
    output.q_ib.coeffs() = s.q_ib.coeffs() * c;
    output.q_bi.coeffs() = s.q_bi.coeffs() * c;
    output.omega_b = s.omega_b * c;
    return output;
}
inline SensorState operator*(double c, const State& s){
    State output;
    output.pos_i = c * s.pos_i;
    output.vel_b = c * s.vel_b;
    output.q_ib = c * s.q_ib.coeffs();
    output.q_bi = c * s.q_bi.coeffs();
    output.omega_b = c * s.omega_b;
    return output;
}
inline SensorState operator+(const State& s1, const State& s2){
    State s_sum;
    s_sum.pos_i = s1.pos_i + s2.pos_i;
    s_sum.vel_b = s1.vel_b + s2.vel_b;
    s_sum.q_ib = s1.q_ib.coeffs() + s2.q_ib.coeffs();
    s_sum.q_bi = s1.q_bi.coeffs() + s2.q_bi.coeffs();
    s_sum.omega_b = s1.omega_b + s2.omega_b;
    return s_sum;
}