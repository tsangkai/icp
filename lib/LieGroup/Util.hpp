
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <concepts>

inline Eigen::Matrix3d cross(const Eigen::Vector3d &vec) {
    Eigen::Matrix3d mat;
    mat << 0.0, -vec.z(), vec.y(), vec.z(), 0.0, -vec.x(), -vec.y(), vec.x(),
        0.0;
    return mat;
}
// clang-format on

inline Eigen::Matrix<double, 3, 6> jacobian(const Eigen::Matrix3d &R,
                                            const Eigen::Vector3d &p) {
    Eigen::Matrix<double, 3, 6> J1 = Eigen::Matrix<double, 3, 6>::Zero();
    J1.block<3, 3>(0, 0) = R;
    J1.block<3, 3>(0, 3) = -R * cross(p);

    return J1;
}

inline Eigen::Matrix3d Exp(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const normedVec = vec / norm;

    return Eigen::Matrix3d::Identity() + std::sin(norm) * cross(normedVec) +
           (1.0 - std::cos(norm)) * cross(normedVec) * cross(normedVec);
}

inline Eigen::Matrix3d V(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const firstOrderCoeff = (1.0 - std::cos(norm)) / std::pow(norm, 2);
    auto const secondOrderCoeff = (norm - std::sin(norm)) / std::pow(norm, 3);

    return Eigen::Matrix3d::Identity() + firstOrderCoeff * cross(vec) +
           secondOrderCoeff * cross(vec) * cross(vec);
}
