
#pragma once

#include <Eigen/Core>

// (source, targe)
using CorrespondenceSet = std::vector<std::pair<size_t, size_t>>;

struct IcpParameter {
    size_t maxIteration;
    double maxDistance;

    // convergence criterion: the decrease ratio of correspondence number
    double correspondenceRatioThreshold;
};

struct IcpResult {
    Eigen::Matrix4d transformation;
    CorrespondenceSet correspondenceSet;
};

enum class IcpError {
    NO_CORRESPONDENCE,
    NO_TRANSFORMATION,
};
