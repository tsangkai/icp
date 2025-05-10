

#pragma once

#include <Eigen/Core>
#include <nanoflann.hpp>

namespace {

// clang-format off
using KDTreeType = nanoflann::KDTreeEigenMatrixAdaptor<
    /* MatrixType */ const Eigen::MatrixXd,
    /* DIM */ -1,
    /* Distance */ nanoflann::metric_L2_Simple,
    /* row_major */ false>;
// clang-format on

constexpr int LEAF_MAX_SIZE = 15;

}    // namespace

template <class PointCloudType>
class KdTree {
  public:
    KdTree(const PointCloudType &pointCloud)
        : data_(Eigen::Map<const Eigen::MatrixXd>(
              reinterpret_cast<const double *>(pointCloud.points.data()), 3,
              pointCloud.points.size())),
          nanoflannIndex_(std::make_unique<KDTreeType>(data_.rows(), data_,
                                                       LEAF_MAX_SIZE)) {}

    std::vector<size_t> knnSearch(const Eigen::Vector3d &query,
                                  size_t numOfNeighbors) const {
        // TODO: ensure that numOfNeighbors is valid, i.e. numOfNeighbors >0

        auto neighborEigenIndex =
            std::vector<Eigen::MatrixXd::Index>(numOfNeighbors);
        auto squaredDist = std::vector<double>(numOfNeighbors);

        nanoflannIndex_->query(query.data(), numOfNeighbors,
                               &neighborEigenIndex[0], &squaredDist[0]);

        auto neighborIndex = std::vector<size_t>(neighborEigenIndex.size());
        std::transform(neighborEigenIndex.cbegin(), neighborEigenIndex.cend(),
                       neighborIndex.begin(), [](Eigen::MatrixXd::Index idx) {
                           return static_cast<size_t>(idx);
                       });

        return neighborIndex;
    }

  private:
    Eigen::MatrixXd data_;
    std::unique_ptr<KDTreeType> nanoflannIndex_;
};
