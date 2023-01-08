#pragma once

#include <lenny/robot/Utils.h>

#include <functional>

namespace lenny::robot {

class EndEffector {
public:
    //--- Constructor
    EndEffector(const std::string& linkName, const uint& stateSize, const tools::Transformation& localGraspTrafo);
    EndEffector(EndEffector&&) = default;
    ~EndEffector() = default;

    //--- Helpers
    void checkState(const Eigen::VectorXd& state) const;

    //--- Drawing
    void drawScene(const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose, const std::optional<Eigen::Vector3d>& color,
                   const double& alpha) const;
    void drawGraspLocation(const tools::Transformation& globalLinkPose) const;
    void drawGui(const std::string& description);

public:
    //--- Members
    const std::string linkName;
    const uint stateSize;
    tools::Transformation localGraspTrafo;
    std::vector<robot::Visual> visuals;
    std::function<void(const std::vector<robot::Visual>& visuals, const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose,
                       const std::optional<Eigen::Vector3d>& color,
                       const double& alpha)>
        f_drawVisuals = nullptr;  //Default set in constructor
    std::function<void()> f_drawAdditionalGuiContent = nullptr;
};

}  // namespace lenny::robot