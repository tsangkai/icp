
#pragma once
#include <Eigen/Core>

namespace LieGroup {

template <std::floating_point T>
inline Eigen::Matrix<T, 3, 3> cross(const Eigen::Vector<T, 3> &vec) {
    Eigen::Matrix<T, 3, 3> mat = Eigen::Matrix<T, 3, 3>::Zero();
    mat(0, 1) = -vec(2);
    mat(1, 0) = vec(2);
    mat(0, 2) = vec(1);
    mat(2, 0) = -vec(1);
    mat(1, 2) = -vec(0);
    mat(2, 1) = vec(0);

    return mat;
}

}    // namespace LieGroup