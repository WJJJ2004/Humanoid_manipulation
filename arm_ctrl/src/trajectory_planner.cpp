#include "trajectory_planner.hpp"
#include <cmath>

Eigen::Vector3d TrajectoryPlanner::cubicInterp(
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
    double t)
{
    Eigen::Vector3d a0 = p0;
    Eigen::Vector3d a1 = v0;
    Eigen::Vector3d a2 = 3.0 * (p1 - p0) - 2.0 * v0 - v1;
    Eigen::Vector3d a3 = 2.0 * (p0 - p1) + v0 + v1;
    return a0 + a1 * t + a2 * t * t + a3 * t * t * t;
}

std::vector<Eigen::Vector3d> TrajectoryPlanner::generateStraightPushTrajectory(
    const Eigen::Vector3d& target_coord,
    const Eigen::Vector3d& offset_coord,
    double push_yaw,
    double duration,   // seconds
    double dt)         // seconds per step
{
    std::vector<Eigen::Vector3d> path;

    Coord2D projected_point = projectOntoLineWithSlope(
        -std::tan(push_yaw), target_coord.x(),
        target_coord.y(), offset_coord.x(), offset_coord.y());

    Eigen::Vector3d start_point(projected_point.x, projected_point.y, target_coord.z());

    // 구간별 벡터 및 거리
    Eigen::Vector3d offset2start = start_point - offset_coord;
    Eigen::Vector3d start2target = target_coord - start_point;

    double len1 = offset2start.norm();
    double len2 = start2target.norm();

    double v_mag = (len1 + len2) / duration;

    Eigen::Vector3d v0 = offset2start.normalized() * v_mag;
    Eigen::Vector3d v1 = start2target.normalized() * v_mag;

    int steps = static_cast<int>(duration / dt);
    double half_duration = duration / 2.0;

    // offset2start (0 ≤ t ≤ 0.5)
    for (int i = 0; i <= steps / 2; ++i) {
        double t = static_cast<double>(i) / (steps / 2);  // normalize to [0,1]
        Eigen::Vector3d p = cubicInterp(offset_coord, v0, start_point, v1, t);
        path.push_back(p);
    }

    // start2target (0.5 ≤ t ≤ 1.0)
    for (int i = 1; i <= steps / 2; ++i) {
        double t = static_cast<double>(i) / (steps / 2);  // normalize to [0,1]
        Eigen::Vector3d p = cubicInterp(start_point, v1, target_coord, Eigen::Vector3d::Zero(), t);
        path.push_back(p);
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