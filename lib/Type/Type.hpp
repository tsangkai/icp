

#pragma once

#include "PointCloud.hpp"
#include "Transformation.hpp"

namespace MyType {
using TimePoint = std::chrono::system_clock::time_point;

template <class Type> struct Stamped {
    TimePoint timestamp;
    Type data;
};

}    // namespace MyType
