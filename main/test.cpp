

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <thread>

#include <Eigen/Core>
#include <cxxopts.hpp>
#include <open3d/Open3D.h>

#include "Icp.hpp"
#include "KdTree.hpp"
#include "Type.hpp"

namespace Color {
Eigen::Vector3d Red{1.0, 0.0, 0.0};
Eigen::Vector3d Green{0.0, 1.0, 0.0};
Eigen::Vector3d Blue{0.0, 0.0, 1.0};
}    // namespace Color

int main(int argc, char *argv[]) {

    cxxopts::Options options("MyProgram", "One line description of MyProgram");

    options.add_options()("d,dataset", "Dataset path",
                          cxxopts::value<std::string>());

    auto result = options.parse(argc, argv);

    auto const datasetPath =
        std::filesystem::path{result["dataset"].as<std::string>()};

    auto const pointCloudFolder = std::filesystem::path{"velodyne_points/data"};

    std::set<std::filesystem::path> sorted_by_name;

    for (auto const &entry :
         std::filesystem::directory_iterator{datasetPath / pointCloudFolder})
        sorted_by_name.insert(entry.path());

    std::shared_ptr<MyType::PointCloud> beforePointCloudPtr;
    std::shared_ptr<MyType::PointCloud> afterPointCloudPtr;

    for (auto const &filename : sorted_by_name) {
        std::cout << filename.c_str() << std::endl;

        auto pointCloudPtr = std::make_shared<MyType::PointCloud>();
        pointCloudPtr->readPointCloudFromKitti(filename);

        beforePointCloudPtr = afterPointCloudPtr;
        afterPointCloudPtr = pointCloudPtr;

        if (beforePointCloudPtr && afterPointCloudPtr) {

            // remove ground points
            beforePointCloudPtr->removeGroundPoint();
            afterPointCloudPtr->removeGroundPoint();

            auto o3dBeforePointCloudPtr =
                beforePointCloudPtr->toOpen3dPointCloud();
            auto o3dAfterPointCloudPtr =
                afterPointCloudPtr->toOpen3dPointCloud();

            o3dBeforePointCloudPtr->PaintUniformColor({1.0, 0.0, 0.0});
            o3dAfterPointCloudPtr->PaintUniformColor({0.0, 1.0, 0.0});

            IcpRunner<PointToPlaneIcp> icpRunner;

            auto const maybeIcpResult = icpRunner.run(
                *beforePointCloudPtr, *afterPointCloudPtr,
                MyType::Transformation::Identity(), {10U, 0.5, 0.01});

            auto const &icpResult = maybeIcpResult.value();

            std::vector<size_t> sourceIndicies(
                icpResult.correspondenceSet.size());
            std::vector<size_t> targetIndicies(
                icpResult.correspondenceSet.size());

            for (auto const &[sourceIdx, targetIdx] :
                 icpResult.correspondenceSet) {
                sourceIndicies.push_back(sourceIdx);
                targetIndicies.push_back(targetIdx);
            }

            auto sourcePointCloudPtr =
                o3dBeforePointCloudPtr->SelectByIndex(sourceIndicies);
            auto targetPointCloudPtr =
                o3dAfterPointCloudPtr->SelectByIndex(targetIndicies);

            // open3d::visualization::DrawGeometries(
            //     {sourcePointCloudPtr, targetPointCloudPtr},
            //     filename.filename(), 1600, 900);

            // open3d::visualization::DrawGeometries(
            //     {o3dBeforePointCloudPtr, o3dAfterPointCloudPtr},
            //     filename.filename(), 1600, 900);

            auto transformBeforePointCloudPtr =
                std::make_shared<open3d::geometry::PointCloud>(
                    o3dBeforePointCloudPtr->Transform(
                        icpResult.transformation.matrix()));
            open3d::visualization::DrawGeometries(
                {transformBeforePointCloudPtr, o3dAfterPointCloudPtr},
                " transform", 1600, 900);
        }
    }

    return 0;
}
