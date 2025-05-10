

#pragma once

#include <Eigen/Core>
#include <open3d/Open3D.h>

namespace MyType {

struct PointCloud {
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> normals;
    std::vector<double> intensities;

    // TODO: better interface
    void addPoint(std::array<float, 4> data) {
        points.emplace_back(data[0], data[1], data[2]);
        intensities.emplace_back(data[3]);
    }

    // TODO: better interface
    void readPointCloudFromKitti(std::string binFilePath) {
        std::fstream input(binFilePath.c_str(),
                           std::ios::in | std::ios::binary);

        if (!input.good()) {
            std::cerr << "Could not read file: " << binFilePath << std::endl;
            exit(EXIT_FAILURE);
        }

        input.seekg(0, std::ios::beg);

        for (int i = 0; input.good() && !input.eof(); i++) {
            std::array<float, 4> data;
            input.read(reinterpret_cast<char *>(data.data()),
                       4 * sizeof(float));

            addPoint(data);
        }
        input.close();

        //
        auto const tempPointCloudPtr = toOpen3dPointCloud();
        tempPointCloudPtr->EstimateNormals();
        tempPointCloudPtr->NormalizeNormals();
        normals = tempPointCloudPtr->normals_;
        //
    }

    std::shared_ptr<open3d::geometry::PointCloud> toOpen3dPointCloud() const {
        auto pointCloudPtr =
            std::make_shared<open3d::geometry::PointCloud>(points);
        pointCloudPtr->normals_ = normals;
        return pointCloudPtr;
    }
};

}    // namespace MyType