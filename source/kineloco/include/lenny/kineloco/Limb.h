#pragma once

#include <lenny/kineloco/ContactPhase.h>
#include <lenny/kineloco/LimbInfo.h>
#include <lenny/tools/Trajectory.h>

namespace lenny::kineloco {

class Limb : public LimbInfo {
public:
    Limb(const LimbInfo& info, const Eigen::Vector3d& defaultOffset)
        : LimbInfo(info), defaultOffset(defaultOffset) {}
    ~Limb() = default;

    void reset();
    ContactPhase getContactPhaseForTime(const double& time) const;
    void drawGui();

public:
    const Eigen::Vector3d defaultOffset;
    tools::Trajectory3d trajectory;
    std::vector<std::pair<double, double>> swingPhases;  //[start, end]
};

}  // namespace lenny::kineloco