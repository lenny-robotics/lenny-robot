#pragma once

#include <lenny/robot/Utils.h>
#include <lenny/tools/Typedefs.h>

namespace lenny::robot {

class EndEffector {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(EndEffector)

    //--- Constructor
    EndEffector(const std::string& linkName, const tools::Transformation& localGraspTrafo);
    virtual ~EndEffector() = default;

    //--- Helpers
    virtual uint getStateSize() const;
    void checkState(const Eigen::VectorXd& state) const;

    //--- Drawing
    virtual void drawScene(const Eigen::VectorXd& state, const tools::Transformation& globalLinkPose, const std::optional<Eigen::Vector3d>& color,
                           const double& alpha, const bool& showGraspLocation) const;
    void drawGui(const std::string& description);

protected:
    void drawGraspLocation(const tools::Transformation& globalLinkPose) const;
    virtual void drawAdditionalGuiContent() {}

public:
    //--- Members
    const std::string linkName;
    tools::Transformation localGraspTrafo;
    std::vector<robot::Visual> visuals;
};

}  // namespace lenny::robot