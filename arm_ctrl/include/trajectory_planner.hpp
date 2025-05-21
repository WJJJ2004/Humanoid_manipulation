#pragma once

#include <vector>
#include <Eigen/Dense>
#include "ik_module.hpp"

class TrajectoryPlanner {
public:
    std::vector<Eigen::Vector3d> generateVerticalPushTrajectory(
    const Eigen::Vector3d& target_point,
    int steps);

private:
  Eigen::Vector3d q_offset_;
  std::shared_ptr<IKModule> ik_module_;
};
