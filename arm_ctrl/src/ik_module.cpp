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
    node->get_parameter("offset_theta1", offset_angle.x());
    node->get_parameter("offset_theta2", offset_angle.y());
    node->get_parameter("offset_theta3", offset_angle.z());

    offset_coord = forwardKinematics(offset_angle).block<3,1>(0,3);

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
    if(Reachable_count > 0) return true; //추후 threshold 조정 필요
    else if(temp.has_value()) return true; // IK가 성공적으로 값을 반환한 경우
    else return false;
}

std::optional<Eigen::Vector3d> IKModule::computeIK(const Eigen::Vector3d& target_position, bool is_left_arm) {
    const double tol = 1e-3;
    const int max_iter = 200;

    Eigen::Vector3d target = target_position;
    if (is_left_arm)
        target.y() *= -1;  // 좌우 반전 고려

    // 💡 초기값: 좌우 팔 대칭 구조를 반영
    Eigen::Vector3d q = is_left_arm ?
        Eigen::Vector3d(-M_PI_4, M_PI_4, -M_PI_4) :
        Eigen::Vector3d(M_PI_4, M_PI_4, M_PI_4);

    // 초기 Jacobian과 위치 계산
    Eigen::Matrix3d J0 = computeJacobian(q);
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> cod(J0);
    Eigen::Matrix3d J_pinv = cod.pseudoInverse();  // Pseudo-Inverse로 발산 방지

    Eigen::Vector3d current_pos = forwardKinematics(q).block<3,1>(0,3);

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector3d error = target - current_pos;

        if (error.norm() < tol) {
            std::cerr << "[IK] Converged in " << i << " iterations.\n";

            // 클램핑 조인트 제한 적용 (안정성)
            for (int j = 0; j < 3; ++j)
                q[j] = std::clamp(q[j], min_angle[j], max_angle[j]);

            return q;
        }

        // 갱신된 Jacobian (특이점 가까울 경우 다시 계산)
        Eigen::Matrix3d J = computeJacobian(q);

        // 특이점 근처이면 이전 pseudo-inverse 재사용
        if (std::abs(J.determinant()) < 1e-6) {
            std::cerr << "[IK] Near singularity. Reusing previous Jacobian.\n";
        } else {
            J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
        }

        Eigen::Vector3d dq = J_pinv * error;
        q += dq;

        // 업데이트된 위치
        current_pos = forwardKinematics(q).block<3,1>(0,3);
    }

    std::cerr << "[IK] Failed to converge to target: ("
              << target.x() << ", " << target.y() << ", " << target.z() << ")\n";

    return std::nullopt;
}

// std::vector<Eigen::Vector3d> IKModule::computeIKTrajectory(
//     const std::vector<Eigen::Vector3d>& trajectory_points,
//     bool is_left_arm
// ) {
//     std::vector<Eigen::Vector3d> joint_trajectory;

//     if (trajectory_points.empty()) {
//         RCLCPP_WARN(rclcpp::get_logger("IKModule"), "Trajectory is empty.");
//         return joint_trajectory;
//     }

//     // 초기 자세: 첫 번째 포인트 기준 IK 수행
//     std::optional<Eigen::Vector3d> q_opt = computeIK(trajectory_points.front(), is_left_arm);
//     if (!q_opt.has_value()) {
//         RCLCPP_ERROR(rclcpp::get_logger("IKModule"), "Failed to initialize IK for first point.");
//         return joint_trajectory;
//     }

//     Eigen::Vector3d q_prev = q_opt.value();
//     joint_trajectory.push_back(q_prev);

//     // 각 trajectory 포인트에 대해 반복 수행
//     for (size_t i = 1; i < trajectory_points.size(); ++i) {
//         const Eigen::Vector3d& target = trajectory_points[i];

//         Eigen::Vector3d current_pos = forwardKinematics(q_prev).block<3,1>(0,3);
//         Eigen::Vector3d error = target;
//         if (is_left_arm)
//             error.y() *= -1;
//         error -= current_pos;

//         Eigen::Matrix3d J = computeJacobian(q_prev);

//         // pseudo-inverse 적용
//         Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix3d> cod(J);
//         Eigen::Matrix3d J_pinv = cod.pseudoInverse();

//         Eigen::Vector3d dq = J_pinv * error;
//         Eigen::Vector3d q_next = q_prev + dq;

//         // 각도 제한 적용
//         for (int j = 0; j < 3; ++j)
//             q_next[j] = std::clamp(q_next[j], min_angle[j], max_angle[j]);

//         joint_trajectory.push_back(q_next);
//         q_prev = q_next;
//     }

//     return joint_trajectory;
// }

