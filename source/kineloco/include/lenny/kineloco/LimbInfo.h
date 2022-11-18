#pragma once

#include <lenny/tools/Definitions.h>

namespace lenny::kineloco {

class LimbInfo {
public:
    LimbInfo(const std::string& linkName, const Eigen::Vector3d& localPosition, const std::pair<double, double>& timeInterval);
    LimbInfo(const LimbInfo&) = default;
    ~LimbInfo() = default;

public:
    const std::string linkName;
    Eigen::Vector3d localPosition;
    std::pair<double, double> timeInterval;
};

}  // namespace lenny::kineloco