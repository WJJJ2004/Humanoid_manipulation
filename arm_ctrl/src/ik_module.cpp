#include "ik_module.hpp"
#include <cmath>
#include <iostream>

IKModule::IKModule(rclcpp::Node* node) {
    node->get_parameter("L1", L1);
    node->get_parameter("L2", L2);
    node->get_parameter("L3", L3);
    node->get_parameter("L4", L4);
    node->get_parameter("d1", d1);
    node->get_parameter("d2", d2);

    std::cerr << "[IK] L1: " << L1 << ", L2: " << L2 << ", L3: " << L3 << ", L4: " << L4 << "\n";
    std::cerr << "[IK] d1: " << d1 << ", d2: " << d2 << "\n";
    std::cerr << "[IK] Initialized IKModule.\n";
}

Eigen::Matrix4d IKModule::computeDH(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta), 0, a,
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha),
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha),
         0, 0, 0, 1;
    return T;
}

Eigen::Matrix4d IKModule::forwardKinematics(const Eigen::Vector3d& q) {
    double th1 = q[0], th2 = q[1], th3 = q[2];
    Eigen::Matrix4d T1 = computeDH(0, 0, -d1, 0);
    Eigen::Matrix4d T2 = computeDH(0, M_PI_2, L1, th1);
    Eigen::Matrix4d T3 = computeDH(0, 0, L2, -M_PI_2);
    Eigen::Matrix4d T4 = computeDH(d2, -M_PI_2, 0, th2);
    Eigen::Matrix4d T5 = computeDH(0, -M_PI_2, 0, M_PI_2);
    Eigen::Matrix4d T6 = computeDH(0, -M_PI_2, 0, M_PI_2);
    Eigen::Matrix4d T7 = computeDH(L3, 0, 0, th3);
    Eigen::Matrix4d T8 = computeDH(L4, 0, 0, 0);
    return T1 * T2 * T3 * T4 * T5 * T6 * T7 * T8;
}

Eigen::Matrix3d IKModule::computeJacobian(const Eigen::Vector3d& q) {
    double delta = 1e-5;
    Eigen::Vector3d f0 = forwardKinematics(q).block<3,1>(0,3);
    Eigen::Matrix3d J;
    for (int i = 0; i < 3; ++i) {
        Eigen::Vector3d dq = q;
        dq[i] += delta;
        Eigen::Vector3d fi = forwardKinematics(dq).block<3,1>(0,3);
        J.col(i) = (fi - f0) / delta;
    }
    return J;
}

bool IKModule::isReachable(const Eigen::Vector3d& target_position, bool is_left_arm) {

    auto temp = computeIK(target_position,is_left_arm);
    if(Reachable_count > 10) return true;
    else return false;
}

std::optional<Eigen::Vector3d> IKModule::computeIK(const Eigen::Vector3d& target_position, bool is_left_arm) {
    Eigen::Vector3d q = Eigen::Vector3d(M_PI_4, M_PI_4, M_PI_4);
    const double tol = 1e-3;
    const int max_iter = 200;
    Eigen::Vector3d target = target_position;
    if (is_left_arm) target.y() *= -1;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d current_pos = forwardKinematics(q).block<3,1>(0,3);
        Eigen::Vector3d error = target - current_pos;
        if (error.norm() < tol) {
            std::cerr << "[IK] Converged to solution.\n";
            Reachable_count ++;
            return q;
        }
        Eigen::Matrix3d J = computeJacobian(q);
        Eigen::Vector3d dq = J.inverse() * error;
        q += dq;
    }

    std::cerr << "[IK] Failed to converge.\n";
    Reachable_count = 0;
    return std::nullopt;
}
