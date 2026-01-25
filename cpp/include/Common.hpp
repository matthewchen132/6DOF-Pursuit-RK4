#pragma once
#include <vector>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <functional>

using State6 = Eigen::Matrix<double,6,1>;
using dXdt_6 = std::function<State6(double T, State6)>;