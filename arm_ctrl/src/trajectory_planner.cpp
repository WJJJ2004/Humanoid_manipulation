#include "trajectory_planner.hpp"
#include <cmath>

std::vector<Eigen::Vector3d> TrajectoryPlanner::generateStraightPushTrajectory(
    const Eigen::Vector3d& target_coord,
    const Eigen::Vector3d& offset_coord,
    double push_yaw,
    int steps)
{
  std::vector<Eigen::Vector3d> path;
  Coord2D projected_point = projectOntoLineWithSlope(
      - std::tan(push_yaw), target_coord.x(),
      target_coord.y(), offset_coord.x(), offset_coord.y());

  Eigen::Vector3d start_point = {projected_point.x,projected_point.y,target_coord.z()};

  double offset2start_length = size(offset_coord - start_point);
  Eigen::Vector3d offset2start_unitVector = (start_point - offset_coord) / offset2start_length;

  Eigen::Vector3d start2EE_unitVector(std::cos(push_yaw), std::sin(push_yaw), 0.0);

  double offset2start_stepSize = offset2start_length / static_cast<double>(steps);
  double start2EE_stepSize = size(target_coord - start_point) / static_cast<double>(steps);

  for (int i = 0; i <= steps; ++i) // offset 지점 부터 start point 까지 선형 보간
  {
    path.push_back(offset_coord + offset2start_unitVector * offset2start_stepSize * i);
  }

  for (int i = 1; i <= steps; ++i)  // start point 부터 end point 까지 선형 보간
  {
    path.push_back(start_point + start2EE_unitVector * start2EE_stepSize * i);
  }

  return path;
}

TrajectoryPlanner::Coord2D TrajectoryPlanner::projectOntoLineWithSlope(
    double m, double x1, double y1, double x0, double y0)
{
    double x_p = (m * (y0 - y1) + x0 + m * m * x1) / (m * m + 1);
    double y_p = y1 + m * (x_p - x1);
    return {x_p, y_p};
}
