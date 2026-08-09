#pragma once
#include <vector>

/*
Features Required to Model:
[] Noise propagation
[] Rotations / Translations to map image (T, K)
[] 

Assumptions:
 - Stereo Camera with multiple images (allows triangulation)
*/


// struct Image() {
//     // -- Raw data from an Image / snapsho --
//     Eigen::MatrixXd pixels = Eigen::MatrixXd(); // Represents all the points in current image
// };

// class component::Camera(Eigen::Vector3d&) {
//     public:
//         // -- Initialize the Camera --
//         Camera(Eigen::Vector3d& camera_xyz_cb) : { r_cb = camera_xyz_cb}

//         // -- Rotation matrices to map Camera -> Body.

//         // -- Estimated state --
//         State X_vbn_curr = generate_state()      // 6DOF State estimate

//         // == HELPER FUNCTIONS ==
//         void pixels_to_unit_ray(Image& cam_readin) {} // <- Converts raw Image data to rays
//         void compute_State(State X_prev, State X_curr){}
//         void compute_delta_X(){}

//         void linear_extraction() {} // <- draws a line and sees whether the points are outliers from the data set

//     private:
//         // -- Settings --
//         const bool feature_based = true; // Defaults feature_based to true (VIO / RELATIVE VIO)
//         const bool key_framing = false;

//         // -- CORE DATA STORED --
//         std::vector<Image> images;
//         Eigen::Vector3d r_cb;
//         Eigen::Vector4d T;

//         // -- Sampling times, delays (Shared between ALL Cameras) --
//         static constexpr int sampling_freq_hz = 50;  // Sampling rate (hz)
//         static constexpr double reading_delay_ms = 0.0; // delay between data and time.

//         // -- NOISE -- (VO Noise is approximately white -> Integration leads to random walk)
//         gyro_white_dist (0.0, ARW / std::sqrt(dt)),
//         accel_white_dist(0.0, VRW / std::sqrt(dt)),
// };


















