
#pragma once

// IDEA: remove PointCloud type dependency, and make ICP templated

#include "KdTree.hpp"
#include "Type.hpp"

#include <Eigen/Cholesky> 

#include <numeric>
#include <cmath>

// (source, targe)
using CorrespondenceSet = std::vector<std::pair<size_t, size_t>>;

struct IcpResult {
    Eigen::Matrix4d transformation;
    CorrespondenceSet correspondenceSet;
};


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

Eigen::Matrix3d Exp(const Eigen::Vector3d& vec) {
    auto const norm = vec.norm();
    Eigen::Vector3d normedVec = vec / norm;

    return Eigen::Matrix3d::Identity() + std::sin(norm) * cross(normedVec) + (1.0 - std::cos(norm)) * cross(normedVec) * cross(normedVec);
}


Eigen::Matrix3d V(const Eigen::Vector3d& vec) {
    auto const norm = vec.norm();
    auto const firstOrderCoeff = (1.0 - std::cos(norm)) / std::pow(norm, 2);
    auto const secondOrderCoeff = (norm - std::sin(norm)) / std::pow(norm, 3);

    return Eigen::Matrix3d::Identity() + firstOrderCoeff * cross(vec) + secondOrderCoeff * cross(vec) * cross(vec);
}

Eigen::Matrix4d findTransformation(const MyType::PointCloud &sourcePointCloud,
                                   const MyType::PointCloud &targetPointCloud,
                                   const Eigen::Matrix4d &transformation,
                                   const CorrespondenceSet &correspondenceSet) {

    if (correspondenceSet.empty()) {
        return transformation;
    }


    // The linear equation: A X = b
    Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

    for (auto const &[sourceIdx, targetIdx] : correspondenceSet) {

        auto const& sourcePoint = sourcePointCloud.points[sourceIdx];
        auto const& targetPoint = targetPointCloud.points[targetIdx];
        auto const targetNormalT = targetPointCloud.normals[targetIdx].transpose();
        auto const transformedSourcePoint = transformation.block<3, 3>(0, 0) * sourcePoint + transformation.block<3, 1>(0, 3);


        Eigen::Matrix<double, 3, 6> J1;
        J1.block<3, 3>(0, 0) = transformation.block<3, 3>(0, 0);
        J1.block<3, 3>(0, 3) = -transformation.block<3, 3>(0, 0) * cross(sourcePoint);

        const Eigen::Matrix<double, 1, 6> J = targetNormalT * J1;

        double const f = targetNormalT * (transformedSourcePoint - targetPoint);

        A += (J.transpose() * J) / (correspondenceSet.size());
        b += -(J.transpose() * f ) / (correspondenceSet.size());

    }


    std::cout << "A = \n" << A << std::endl;
    std::cout << "b = \n" << b << std::endl;

    Eigen::Vector<double, 6> const dT = A.ldlt().solve(b);
    Eigen::Vector3d rho = dT.segment<3>(0);
    Eigen::Vector3d theta = dT.segment<3>(3);

    Eigen::Matrix4d exp_dt = Eigen::Matrix4d::Identity();
    exp_dt.block<3,3>(0, 0) = Exp(theta);
    exp_dt.block<3,1>(0, 3) = V(theta) * rho;

    return transformation * exp_dt;
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

    constexpr int MAX_ITERATION = 5;  // 15 for point to point
    Eigen::Matrix4d transformation = initTransformation;
    CorrespondenceSet correspondenceSet;

    for (int i = 0; i < MAX_ITERATION; ++i) {
        correspondenceSet = findCorrespondence(
            sourcePointCloud, targetPointCloud, transformation, 0.5);

        auto updateTransformation = findTransformation(sourcePointCloud, targetPointCloud,
                                            transformation,
                                            correspondenceSet);

        std::cout << "iteration: " << i << std::endl;
        std::cout << "transformation: \n" << updateTransformation << std::endl;
        std::cout << "correspondence size: " << correspondenceSet.size()
                  << std::endl;

        transformation = updateTransformation;
    }

    return {transformation, correspondenceSet};
}
