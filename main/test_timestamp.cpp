

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <set>
#include <thread>

#include <fstream>
#include <string>

#include <chrono>
#include <iomanip>
#include <sstream>

#include <Eigen/Core>
#include <cxxopts.hpp>
#include <open3d/Open3D.h>

#include <absl/log/check.h>
#include <absl/log/log.h>

#include "Icp.hpp"
#include "KdTree.hpp"
#include "Type.hpp"

class KittiPointCloudReader {
  public:
    KittiPointCloudReader(std::filesystem::path datasetPath)
        : datasetPath_(datasetPath) {
        readTimestamp();

        auto const dataFolder = std::filesystem::path{"data"};
        auto const dataDirPath = datasetPath / dataFolder;

        for (auto const &entry :
             std::filesystem::directory_iterator{dataDirPath})
            sortedFileNames_.insert(entry.path());

        timestampsIter_ = timestamps_.begin();
        sortedFileNamesIter_ = sortedFileNames_.begin();
    };

    auto read()
        -> std::optional<MyType::Stamped<std::shared_ptr<MyType::PointCloud>>> {
        if (sortedFileNamesIter_ == sortedFileNames_.end()) {
            return std::nullopt;
        }

        auto timestamp = *timestampsIter_;
        auto pointCloudPtr = std::make_shared<MyType::PointCloud>();

        pointCloudPtr->readPointCloudFromKitti(*sortedFileNamesIter_);

        ++timestampsIter_;
        ++sortedFileNamesIter_;

        return MyType::Stamped<std::shared_ptr<MyType::PointCloud>>(
            timestamp, pointCloudPtr);
    };

  private:
    void readTimestamp() {

        auto const timestampFile = std::filesystem::path{"timestamps.txt"};

        auto const filePath = datasetPath_ / timestampFile;

        std::ifstream file(filePath);    // Open the file

        auto constexpr format = "%Y-%m-%d %H:%M:%S";

        CHECK(file.is_open()) << "Unable to open file " << filePath;

        std::string line;
        for (std::string line; std::getline(file, line);) {

            std::istringstream ss(line);
            MyType::TimePoint timepoint;

            ss >> std::chrono::parse(format, timepoint);

            timestamps_.push_back(timepoint);
        }
        file.close();    // Close the file
    }

    std::filesystem::path datasetPath_;

    std::vector<MyType::TimePoint> timestamps_;
    std::vector<MyType::TimePoint>::iterator timestampsIter_;

    std::set<std::filesystem::path> sortedFileNames_;
    std::set<std::filesystem::path>::iterator sortedFileNamesIter_;
};

int main(int argc, char *argv[]) {

    // argparse
    cxxopts::Options options("ICP test", "A small program to run ICP.");
    options.add_options()("d,dataset", "Dataset path",
                          cxxopts::value<std::string>());

    auto result = options.parse(argc, argv);

    auto const datasetPath =
        std::filesystem::path{result["dataset"].as<std::string>()};

    auto const pointCloudFolder = std::filesystem::path{"velodyne_points/"};

    KittiPointCloudReader reader(datasetPath / pointCloudFolder);

    for (auto result = reader.read(); result; result = reader.read()) {

        LOG(INFO) << "point cloud data:";

        LOG(INFO) << "timestamp: " << result.value().timestamp;
        LOG(INFO) << "point number: " << result.value().data->points.size();

        std::cin.get();
    }

    return 0;
}
