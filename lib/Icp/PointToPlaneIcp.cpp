

#include "PointToPlaneIcp.hpp"
#include "KdTree.hpp"

#include <Eigen/Cholesky>
#include <manif/SE3.h>

#pragma omp declare reduction(+ : Eigen::Matrix<double, 6, 6> : omp_out =      \
                                  (omp_out + omp_in).eval())                   \
    initializer(omp_priv = Eigen::Matrix<double, 6, 6>::Zero())
#pragma omp declare reduction(+ : Eigen::Matrix<double, 6, 1> : omp_out =      \
                                  (omp_out + omp_in).eval())                   \
    initializer(omp_priv = Eigen::Matrix<double, 6, 1>::Zero())

namespace {

// clang-format off
inline Eigen::Matrix3d cross(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d mat;
    mat <<      0.0, -vec.z(),  vec.y(),
            vec.z(),      0.0, -vec.x(),
           -vec.y(),  vec.x(),      0.0;
    return mat;
}
// clang-format on

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

}    // namespace

MyType::Transformation PointToPlaneIcp::findTransformation(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const MyType::Transformation &transformation,
    const CorrespondenceSet &correspondenceSet) {

    // The linear equation: A X = b
    auto A = Eigen::Matrix<double, 6, 6>::Zero().eval();
    auto b = Eigen::Matrix<double, 6, 1>::Zero().eval();

    auto const T = manif::SE3d{transformation};

#pragma omp parallel reduction(+ : A, b)
    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        // using manif to calculate the jacobian might be slow
        auto const &sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const &targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT =
            targetPointCloud.normals[targetIdx].transpose();
        auto const transformedSourcePoint = transformation * sourcePoint;

        auto J1 = Eigen::Matrix<double, 3, 6>::Zero().eval();
        J1.block<3, 3>(0, 0) = transformation.rotation();
        J1.block<3, 3>(0, 3) = -transformation.rotation() * cross(sourcePoint);

        auto const J = Eigen::Matrix<double, 1, 6>{targetNormalT * J1};
        auto const f = targetNormalT * (transformedSourcePoint - targetPoint);

        A += (J.transpose() * J) / (correspondenceSet.size());
        b += -(J.transpose() * f) / (correspondenceSet.size());
    }

    auto const dT = manif::SE3Tangentd{A.ldlt().solve(b)};

    return T.rplus(dT).isometry();
}

CorrespondenceSet PointToPlaneIcp::findCorrespondence(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const MyType::Transformation &transformation, double minDistance) {

    auto const targetKdTree = KdTree<MyType::PointCloud>(targetPointCloud);

    auto correspondenceSet = CorrespondenceSet{};
    correspondenceSet.reserve(sourcePointCloud.points.size());

    for (size_t i = 0; i < sourcePointCloud.points.size(); ++i) {
        auto const queryPoint = transformation * sourcePointCloud.points[i];
        auto const resultIdx = targetKdTree.knnSearch(queryPoint, 1);
        auto const distance =
            (queryPoint - targetPointCloud.points[resultIdx[0]]).norm();
        if (distance < minDistance) {
            correspondenceSet.push_back({i, resultIdx[0]});
        }
    }

    correspondenceSet.shrink_to_fit();

    return correspondenceSet;
}
