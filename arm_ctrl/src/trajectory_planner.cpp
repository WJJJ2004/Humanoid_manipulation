#include "trajectory_planner.hpp"
#include <cmath>

std::vector<Eigen::Vector3d> TrajectoryPlanner::generateVerticalPushTrajectory(
    const Eigen::Vector3d& target_point,
    int steps)
{
  std::vector<Eigen::Vector3d> path;

  // EE 시작 위치를 초기 조인트 각도로부터 계산
  Eigen::Vector3d start_point = ik_module_->forwardKinematics(q_offset_).block<3,1>(0,3);

  // z 고정: 타겟의 z로 통일
  double fixed_z = target_point.z();
  start_point.z() = fixed_z;

  // 방향 벡터
  Eigen::Vector3d direction = target_point - start_point;
  direction.z() = 0.0;  // z축 방향 제거
  double total_dist = direction.norm();
  Eigen::Vector3d unit_dir = direction.normalized();

  double step_size = total_dist / static_cast<double>(steps);

  for (int i = 0; i <= steps; ++i) {
    Eigen::Vector3d pt = start_point + unit_dir * (step_size * i);
    pt.z() = fixed_z;
    path.push_back(pt);
  }

  return path;
}
