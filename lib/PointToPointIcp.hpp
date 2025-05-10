
#pragma once

// IDEA: remove PointCloud type dependency, and make ICP templated

#include "KdTree.hpp"
#include "Type.hpp"

#include <Eigen/SVD>

#include <numeric>

// (source, targe)
using CorrespondenceSet = std::vector<std::pair<size_t, size_t>>;

struct IcpResult {
    Eigen::Matrix4d transformation;
    CorrespondenceSet correspondenceSet;
};

// ref: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
Eigen::Matrix4d findTransformation(const MyType::PointCloud &sourcePointCloud,
                                   const MyType::PointCloud &targetPointCloud,
                                   const CorrespondenceSet &correspondenceSet) {

    Eigen::Vector3d sourceCenter = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCenter = Eigen::Vector3d::Zero();

    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {
        sourceCenter +=
            sourcePointCloud.points[sourceIdx] / correspondenceSet.size();
        targetCenter +=
            targetPointCloud.points[targetIdx] / correspondenceSet.size();
    }

    Eigen::Matrix3d S = Eigen::Matrix3d::Zero();
    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {
        S += (sourcePointCloud.points[sourceIdx] - sourceCenter) *
             (targetPointCloud.points[targetIdx] - targetCenter).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(S, Eigen::ComputeFullU |
                                                 Eigen::ComputeFullV);

    double const determinant =
        (svd.matrixV() * svd.matrixU().transpose()).determinant();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    if (determinant < 0) {
        R = svd.matrixV() * Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, -1.0) *
            svd.matrixU().transpose();
    } else {
        R = svd.matrixV() * svd.matrixU().transpose();
    }

    Eigen::Vector3d t = targetCenter - R * sourceCenter;

    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = R;
    transformation.block<3, 1>(0, 3) = t;

    return transformation;
}

CorrespondenceSet findCorrespondence(const MyType::PointCloud &sourcePointCloud,
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

IcpResult IterativeClosestPointAlgorithm(
    const MyType::PointCloud &sourcePointCloud,
    const MyType::PointCloud &targetPointCloud,
    const Eigen::Matrix4d &initTransformation = Eigen::Matrix4d::Identity()) {

    constexpr int MAX_ITERATION = 15;
    Eigen::Matrix4d transformation = initTransformation;
    CorrespondenceSet correspondenceSet;

    for (int i = 0; i < MAX_ITERATION; ++i) {
        correspondenceSet = findCorrespondence(
            sourcePointCloud, targetPointCloud, transformation, 0.5);

        transformation = findTransformation(sourcePointCloud, targetPointCloud,
                                            correspondenceSet);

        std::cout << "iteration: " << i << std::endl;
        std::cout << "transformation: \n" << transformation << std::endl;
        std::cout << "correspondence size: " << correspondenceSet.size()
                  << std::endl;
    }

    return {transformation, correspondenceSet};
}
