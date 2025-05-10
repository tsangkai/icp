

#include "PointToPlaneIcp.hpp"
#include "KdTree.hpp"

#include <Eigen/Cholesky>

#include <cmath>
#include <numeric>

// TODO: move to a Lie group library
namespace {

// clang-format off
Eigen::Matrix3d cross(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;
    // 
    mat <<      0.0, -vec.z(),  vec.y(),
            vec.z(),      0.0, -vec.x(),
           -vec.y(),  vec.x(),      0.0;
    return mat;
}
// clang-format on

Eigen::Matrix3d Exp(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const normedVec = vec / norm;

    return Eigen::Matrix3d::Identity() + std::sin(norm) * cross(normedVec) +
           (1.0 - std::cos(norm)) * cross(normedVec) * cross(normedVec);
}

Eigen::Matrix3d V(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const firstOrderCoeff = (1.0 - std::cos(norm)) / std::pow(norm, 2);
    auto const secondOrderCoeff = (norm - std::sin(norm)) / std::pow(norm, 3);

    return Eigen::Matrix3d::Identity() + firstOrderCoeff * cross(vec) +
           secondOrderCoeff * cross(vec) * cross(vec);
}

}    // namespace

Eigen::Matrix4d PointToPlaneIcp::findTransformation(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const Eigen::Matrix4d &transformation,
    const CorrespondenceSet &correspondenceSet) {

    // The linear equation: A X = b
    auto A = Eigen::Matrix<double, 6, 6>::Zero().eval();
    auto b = Eigen::Matrix<double, 6, 1>::Zero().eval();

    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        auto const &sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const &targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT =
            targetPointCloud.normals[targetIdx].transpose();
        auto const transformedSourcePoint =
            transformation.block<3, 3>(0, 0) * sourcePoint +
            transformation.block<3, 1>(0, 3);

        auto J1 = Eigen::Matrix<double, 3, 6>::Zero().eval();
        J1.block<3, 3>(0, 0) = transformation.block<3, 3>(0, 0);
        J1.block<3, 3>(0, 3) =
            -transformation.block<3, 3>(0, 0) * cross(sourcePoint);

        auto const J = Eigen::Matrix<double, 1, 6>{targetNormalT * J1};
        auto const f = targetNormalT * (transformedSourcePoint - targetPoint);

        A += (J.transpose() * J) / (correspondenceSet.size());
        b += -(J.transpose() * f) / (correspondenceSet.size());
    }

    Eigen::Vector<double, 6> const dT = A.ldlt().solve(b);
    Eigen::Vector3d rho = dT.segment<3>(0);
    Eigen::Vector3d theta = dT.segment<3>(3);

    Eigen::Matrix4d exp_dt = Eigen::Matrix4d::Identity();
    exp_dt.block<3, 3>(0, 0) = Exp(theta);
    exp_dt.block<3, 1>(0, 3) = V(theta) * rho;

    return transformation * exp_dt;
}

CorrespondenceSet
PointToPlaneIcp::findCorrespondence(const MyType::PointCloud &sourcePointCloud,
                                    const MyType::PointCloud &targetPointCloud,
                                    const Eigen::Matrix4d &transformation,
                                    double minDistance) {

    auto const targetKdTree = KdTree<MyType::PointCloud>(targetPointCloud);

    CorrespondenceSet correspondenceSet;

    for (size_t i = 0; i < sourcePointCloud.points.size(); ++i) {
        auto const queryPoint =
            (transformation.block<3, 3>(0, 0) * sourcePointCloud.points[i] +
             transformation.block<3, 1>(0, 3))
                .eval();
        auto const resultIdx = targetKdTree.knnSearch(queryPoint, 1);
        auto const distance =
            (queryPoint - targetPointCloud.points[resultIdx[0]]).norm();
        if (distance < minDistance) {
            correspondenceSet.push_back({i, resultIdx[0]});
        }
    }

    return correspondenceSet;
}
