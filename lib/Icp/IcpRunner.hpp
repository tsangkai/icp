
#pragma once

#include "IcpCommon.hpp"
#include "Type.hpp"
#include <chrono>
#include <expected>
#include <iostream>

template <typename T>
concept IcpConcept = requires(T a, const MyType::PointCloud &pointCloud,
                              const MyType::Transformation &transformation,
                              CorrespondenceSet correspondenceSet,
                              double distance) {
    // specify findCorrespondence()
    {
        T::findCorrespondence(pointCloud, pointCloud, transformation, distance)
    } -> std::convertible_to<CorrespondenceSet>;

    // specify findCorrespondence()
    {
        T::findTransformation(pointCloud, pointCloud, transformation,
                              correspondenceSet)
    } -> std::convertible_to<MyType::Transformation>;
};

template <typename IcpType>
    requires IcpConcept<IcpType>
class IcpRunner {

  public:
    std::expected<IcpResult, IcpError>
    run(const MyType::PointCloud &sourcePointCloud,
        const MyType::PointCloud &targetPointCloud,
        const MyType::Transformation &initTransformation,
        const IcpParameter &parameter) {

        auto transformation = initTransformation;
        auto correspondenceSet = CorrespondenceSet{};
        correspondenceNumberHistory_.clear();

        for (int i = 0; i < parameter.maxIteration; ++i) {

            std::cout << "ICP iteration: " << i << std::endl;

            correspondenceSet = IcpType::findCorrespondence(
                sourcePointCloud, targetPointCloud, transformation,
                parameter.maxDistance);

            if (correspondenceSet.empty()) {
                return std::unexpected(IcpError::NO_CORRESPONDENCE);
            }

            if (!correspondenceNumberHistory_.empty() &&
                correspondenceNumberHistory_.back() <=
                    correspondenceSet.size()) {
                auto const changeRatio =
                    (correspondenceNumberHistory_.back() -
                     correspondenceSet.size()) /
                    static_cast<double>(correspondenceNumberHistory_.back());
                if (changeRatio < parameter.correspondenceRatioThreshold)
                    return IcpResult{transformation, correspondenceSet};
            }
            correspondenceNumberHistory_.push_back(correspondenceSet.size());

            const std::chrono::time_point<std::chrono::system_clock>
                beforeTimestamp = std::chrono::system_clock::now();
            auto const updateTransformation =
                IcpType::findTransformation(sourcePointCloud, targetPointCloud,
                                            transformation, correspondenceSet);
            const std::chrono::time_point<std::chrono::system_clock>
                afterTimestamp = std::chrono::system_clock::now();
            std::cout << "find transformation time: "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             afterTimestamp - beforeTimestamp)
                      << std::endl;

            transformation = updateTransformation;
        }

        return IcpResult{transformation, correspondenceSet};
    }

  private:
    std::vector<size_t> correspondenceNumberHistory_;
};