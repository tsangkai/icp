

#include "PointCloud.hpp"

namespace MyType {

void PointCloud::addPoint(std::array<float, 4> data) {
    points.emplace_back(data[0], data[1], data[2]);
    intensities.emplace_back(data[3]);
}

void PointCloud::readPointCloudFromKitti(std::string binFilePath) {
    std::fstream input(binFilePath.c_str(), std::ios::in | std::ios::binary);

    if (!input.good()) {
        std::cerr << "Could not read file: " << binFilePath << std::endl;
        exit(EXIT_FAILURE);
    }

    input.seekg(0, std::ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++) {
        std::array<float, 4> data;
        input.read(reinterpret_cast<char *>(data.data()), 4 * sizeof(float));

        addPoint(data);
    }
    input.close();
}

std::shared_ptr<open3d::geometry::PointCloud>
PointCloud::toOpen3dPointCloud() const {
    auto pointCloudPtr = std::make_shared<open3d::geometry::PointCloud>(points);
    pointCloudPtr->normals_ = normals;
    return pointCloudPtr;
}

PointCloud PointCloud::transform(const Transformation &transform) {
    PointCloud transformedPointCloud;
    transformedPointCloud.points = points;

#pragma omp parallel for
    for (size_t i = 0; i < points.size(); ++i) {
        transformedPointCloud.points[i] = transform * points[i];
    }

    return transformedPointCloud;
}

void PointCloud::removeGroundPoint() {
    size_t idx = 0;
    size_t size = points.size();

    for (size_t i = 0; i < size; ++i) {
        if (points[i].z() >= 0.05) {
            points[idx] = points[i];
            normals[idx] = normals[i];
            intensities[idx] = intensities[i];
            ++idx;
        }
    }

    points.resize(idx);
    normals.resize(idx);
    intensities.resize(idx);
}

}    // namespace MyType