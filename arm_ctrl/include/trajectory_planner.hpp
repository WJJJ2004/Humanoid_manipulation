#ifndef ARM_CTRL__TRAJECTORY_PLANNER_HPP_
#define ARM_CTRL__TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>
#include <vector>
#include <utility>

class TrajectoryPlanner {
public:
  struct Coord2D {
    double x;
    double y;
  };

  std::vector<Eigen::Vector3d> generateStraightPushTrajectory(
    const Eigen::Vector3d& target_coord,
    const Eigen::Vector3d& offset_coord,
    double push_yaw,
    double duration,   // seconds
    double dt);        // seconds per step

  Coord2D projectOntoLineWithSlope(
    double m, double x1, double y1, double x0, double y0);

  Eigen::Vector3d cubicInterp(
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
    double t);
};

#endif  // ARM_CTRL__TRAJECTORY_PLANNER_HPP_
