

#include "PointToPlaneIcp.hpp"
#include "KdTree.hpp"

#include "SE3.hpp"

#include <Eigen/Cholesky>

MyType::Transformation PointToPlaneIcp::findTransformation(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const MyType::Transformation &transformation,
    const CorrespondenceSet &correspondenceSet) {

    // The linear equation: A X = b
    auto A = Eigen::Matrix<double, 6, 6>::Zero().eval();
    auto b = Eigen::Matrix<double, 6, 1>::Zero().eval();

    auto const T = LieGroup::SE3_d{transformation};

    // #pragma omp parallel reduction(+ : A, b)
    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        auto const &sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const &targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT =
            targetPointCloud.normals[targetIdx].transpose();
        auto const transformedSourcePoint = transformation * sourcePoint;

        auto const J =
            (targetNormalT * LieGroup::jacobbian_Mp_to_M(T, sourcePoint))
                .eval();
        auto const f = targetNormalT * (transformedSourcePoint - targetPoint);

        A += (J.transpose() * J) / (correspondenceSet.size());
        b += -(J.transpose() * f) / (correspondenceSet.size());
    }

    auto const dT = LieGroup::SE3Algb_d{A.ldlt().solve(b)};

    return (T + dT).data();
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
