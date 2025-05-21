

#pragma once

#include "Transformation.hpp"
#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace MyType {

class PointCloud {
  public:
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> intensities;

    // TODO: better interface
    void readPointCloudFromKitti(std::string binFilePath);

    std::shared_ptr<open3d::geometry::PointCloud> toOpen3dPointCloud() const;

    PointCloud transform(const Transformation &transform);

    // TODO: very heuristic for the first version
    void removeGroundPoint();

    void estimateNormals();

  private:
    // TODO: better interface
    void addPoint(std::array<float, 4> data);
};

}    // namespace MyType