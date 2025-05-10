

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <thread>

#include <Eigen/Core>
#include <open3d/Open3D.h>

#include "Icp.hpp"
#include "KdTree.hpp"
#include "Type.hpp"

namespace Color {
Eigen::Vector3d Red{1.0, 0.0, 0.0};
Eigen::Vector3d Green{0.0, 1.0, 0.0};
Eigen::Vector3d Blue{0.0, 0.0, 1.0};
}    // namespace Color

// TODO: very heuristic for the first version
void removeGroundPoint(std::shared_ptr<MyType::PointCloud> pointCloudPtr) {
    size_t idx = 0;
    size_t size = pointCloudPtr->points.size();

    for (size_t i = 0; i < size; ++i) {
        if (pointCloudPtr->points[i].z() >= 0.05) {
            pointCloudPtr->points[idx] = pointCloudPtr->points[i];
            pointCloudPtr->normals[idx] = pointCloudPtr->normals[i];
            pointCloudPtr->intensities[idx] = pointCloudPtr->intensities[i];
            ++idx;
        }
    }

    pointCloudPtr->points.resize(idx);
    pointCloudPtr->normals.resize(idx);
    pointCloudPtr->intensities.resize(idx);
}

int main(int argc, char *argv[]) {
    auto const datasetPath = "/home/tsangkai/dataset/kitti/2011_10_03/"
                             "2011_10_03_drive_0027_sync/velodyne_points/data";
    auto const pointCloudData = std::filesystem::path{datasetPath};

    std::set<std::filesystem::path> sorted_by_name;

    for (auto const &entry :
         std::filesystem::directory_iterator{pointCloudData})
        sorted_by_name.insert(entry.path());

    std::shared_ptr<MyType::PointCloud> beforePointCloudPtr;
    std::shared_ptr<MyType::PointCloud> afterPointCloudPtr;

    for (auto const &filename : sorted_by_name) {
        std::cout << filename.c_str() << std::endl;

        auto pointCloudPtr = std::make_shared<MyType::PointCloud>();
        pointCloudPtr->readPointCloudFromKitti(filename);

        beforePointCloudPtr = afterPointCloudPtr;
        afterPointCloudPtr = pointCloudPtr;

        // testing knn search
        /*
        {
            auto const kdTree = KdTree(*pointCloudPtr);
            const size_t neighborSize = 50;

            const auto neighbors =
                kdTree.knnSearch(pointCloudPtr->points.back(), neighborSize);

            auto o3dPointCloudPtr = pointCloudPtr->toOpen3dPointCloud();

            o3dPointCloudPtr->PaintUniformColor(Color::Red);

            for (auto idx : neighbors) {
                o3dPointCloudPtr->colors_.at(idx) = Color::Green;
            }

            open3d::visualization::DrawGeometries(
                {o3dPointCloudPtr}, filename.filename(), 1600, 900);
        }
        */

        if (beforePointCloudPtr && afterPointCloudPtr) {

            // remove ground points
            removeGroundPoint(beforePointCloudPtr);
            removeGroundPoint(afterPointCloudPtr);

            auto o3dBeforePointCloudPtr =
                beforePointCloudPtr->toOpen3dPointCloud();
            auto o3dAfterPointCloudPtr =
                afterPointCloudPtr->toOpen3dPointCloud();

            o3dBeforePointCloudPtr->PaintUniformColor({1.0, 0.0, 0.0});
            o3dAfterPointCloudPtr->PaintUniformColor({0.0, 1.0, 0.0});

            Eigen::Matrix4d initTransfrom = Eigen::Matrix4d::Identity();
            initTransfrom.block<3, 1>(0, 3) = 0.0001 * Eigen::Vector3d::Ones();

            auto const icpResult = IterativeClosestPointAlgorithm(
                *beforePointCloudPtr, *afterPointCloudPtr, initTransfrom);

            /*
            Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
            CorrespondenceSet correspondenceSet;
            for (int i = 0; i < 15; ++i) {
                correspondenceSet = findCorrespondence(*beforePointCloudPtr,
                                                       *afterPointCloudPtr,
                                                       transformation, 0.5);

                transformation =
                    findTransformation(*beforePointCloudPtr,
                                       *afterPointCloudPtr, correspondenceSet);

                std::cout << "iteration: " << i << std::endl;
                std::cout << "transformation: \n"
                          << transformation << std::endl;
                std::cout << "correspondence size: " << correspondenceSet.size()
                          << std::endl;
            }
            */

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


            open3d::visualization::DrawGeometries(
                {sourcePointCloudPtr, targetPointCloudPtr}, filename.filename(),
                1600, 900);

            open3d::visualization::DrawGeometries(
                {o3dBeforePointCloudPtr, o3dAfterPointCloudPtr},
                filename.filename(), 1600, 900);

            auto transformBeforePointCloudPtr =
                std::make_shared<open3d::geometry::PointCloud>(
                    o3dBeforePointCloudPtr->Transform(
                        icpResult.transformation));
            open3d::visualization::DrawGeometries(
                {transformBeforePointCloudPtr, o3dAfterPointCloudPtr},
                " transform", 1600, 900);

            // auto const maxCorrespondenceDistance = 5.0;
            // auto const initTransform = Eigen::Matrix4d::Identity().eval();
            // auto const registrationResult =
            //     open3d::pipelines::registration::RegistrationICP(
            //         *o3dBeforePointCloudPtr, *o3dAfterPointCloudPtr,
            //         maxCorrespondenceDistance, initTransform);

            // o3dBeforePointCloudPtr->Transform(
            //     registrationResult.transformation_);

            // open3d::visualization::DrawGeometries(
            //     {o3dBeforePointCloudPtr, o3dAfterPointCloudPtr},
            //     filename.filename(), 1600, 900);
        }
    }

    return 0;
}
