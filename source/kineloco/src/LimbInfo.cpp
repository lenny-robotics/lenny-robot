#include <lenny/kineloco/LimbInfo.h>
#include <lenny/tools/Logger.h>

namespace lenny::kineloco {

LimbInfo::LimbInfo(const std::string& linkName, const Eigen::Vector3d& localPosition, const std::pair<double, double>& timeInterval)
    : linkName(linkName), localPosition(localPosition), timeInterval(timeInterval) {
    auto checkTimeValue = [](double& val) -> void {
        if (val < 0.0 || val > 1.0)
            LENNY_LOG_WARNING("Trying to set value outside of bounds: %lf -> bounding to range...")
        tools::utils::boundToRange(val, 0.0, 1.0);
    };
    checkTimeValue(this->timeInterval.first);
    checkTimeValue(this->timeInterval.second);
    if (this->timeInterval.first > this->timeInterval.second) {
        LENNY_LOG_WARNING("Lower value is larger than upper value -> setting lower value to upper value")
        this->timeInterval.first = this->timeInterval.second;
    }
}

}  // namespace lenny::kineloco