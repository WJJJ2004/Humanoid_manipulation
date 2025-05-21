#ifndef ARM_CTRL__IK_MODULE_HPP_
#define ARM_CTRL__IK_MODULE_HPP_

#include <memory>          // for std::shared_ptr
#include "rclcpp/rclcpp.hpp"  // for rclcpp::Node
#include <Eigen/Dense>
#include <optional>

class IKModule {
public:
  explicit IKModule(rclcpp::Node* node);

  // Main IK solver (좌우팔 여부 지원)
  std::optional<Eigen::Vector3d> computeIK(const Eigen::Vector3d& target_position, bool is_left_arm = false);

  // EE workspace 내 존재 여부 판정
  bool isReachable(const Eigen::Vector3d& target_position, bool is_left_arm = false);

private:
  double L1, L2, L3, L4, d1, d2;

  int Reachable_count = 0;

  Eigen::Matrix4d computeDH(double a, double alpha, double d, double theta);
  Eigen::Matrix4d forwardKinematics(const Eigen::Vector3d& q);
  Eigen::Matrix3d computeJacobian(const Eigen::Vector3d& q);
};

#endif  // ARM_CTRL__IK_MODULE_HPP_
