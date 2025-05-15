
#pragma once
#include "IcpCommon.hpp"

#include "Type.hpp"

class PointToPlaneIcp {
  public:
    static MyType::Transformation
    findTransformation(const MyType::PointCloud &sourcePointCloud,
                       const MyType::PointCloud &targetPointCloud,
                       const MyType::Transformation &transformation,
                       const CorrespondenceSet &correspondenceSet);

    static CorrespondenceSet
    findCorrespondence(const MyType::PointCloud &sourcePointCloud,
                       const MyType::PointCloud &targetPointCloud,
                       const MyType::Transformation &transformation,
                       double minDistance);
};
