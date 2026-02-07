#include <Eigen/Dense>

struct StateFull {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;        // position
  Eigen::Vector3d v;        // velocity
  Eigen::Quaterniond q;     // attitude (body->world, pick a convention and stick to it)
};
