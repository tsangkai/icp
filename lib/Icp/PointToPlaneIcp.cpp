

#include "PointToPlaneIcp.hpp"
#include "KdTree.hpp"

#include <Eigen/Cholesky>
#include <manif/SE3.h>

MyType::Transformation PointToPlaneIcp::findTransformation(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const MyType::Transformation &transformation,
    const CorrespondenceSet &correspondenceSet) {

    // The linear equation: A X = b
    auto A = Eigen::Matrix<double, 6, 6>::Zero().eval();
    auto b = Eigen::Matrix<double, 6, 1>::Zero().eval();

    auto const T = manif::SE3d{transformation};

    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        auto const &sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const &targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT =
            targetPointCloud.normals[targetIdx].transpose();

        // calculate the Jacobian matrices
        auto J_transform = Eigen::Matrix<double, 3, 6>{};
        auto const transformedSourcePoint = T.act(sourcePoint, J_transform);

        auto const &J_product = targetNormalT;

        auto const J = J_product * J_transform;
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

    for (size_t i = 0; i < sourcePointCloud.points.size(); ++i) {
        auto const queryPoint = transformation * sourcePointCloud.points[i];
        auto const resultIdx = targetKdTree.knnSearch(queryPoint, 1);
        auto const distance =
            (queryPoint - targetPointCloud.points[resultIdx[0]]).norm();
        if (distance < minDistance) {
            correspondenceSet.push_back({i, resultIdx[0]});
        }
    }

    return correspondenceSet;
}
