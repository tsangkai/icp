
#pragma once
#include "IcpCommon.hpp"

#include "Type.hpp"

class PointToPlaneIcp {
  public:
    static Eigen::Matrix4d
    findTransformation(const MyType::PointCloud &sourcePointCloud,
                       const MyType::PointCloud &targetPointCloud,
                       const Eigen::Matrix4d &transformation,
                       const CorrespondenceSet &correspondenceSet);

    static CorrespondenceSet
    findCorrespondence(const MyType::PointCloud &sourcePointCloud,
                       const MyType::PointCloud &targetPointCloud,
                       const Eigen::Matrix4d &transformation,
                       double minDistance);
};
