#include <string>
#include <Eigen/Dense>
#pragma once

// Helpers which break if unhealthy, and provide a helper message to print.
// Meant to be WRAPPED OVER values.


/*
TODO
*/ 

struct SimHealth{
    bool healthy = true;
    std::string helper_msg;
};

// SimHealth angular_vel_check(Eigen::Vector3d& ang_vel, ) const{}
//     For omega in ang_vel, check < than 1e3 rads/sec

