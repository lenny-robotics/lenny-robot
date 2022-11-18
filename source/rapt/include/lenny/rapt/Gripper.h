#pragma once

#include <lenny/robot/Robot.h>
#include <lenny/tools/Typedefs.h>

namespace lenny::rapt {

class Gripper {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(Gripper)

    //--- Constructor
    Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo, const std::string& description = "Gripper");
    virtual ~Gripper() = default;

    //--- Drawing and gui
    virtual void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const;
    void drawGui();

protected:
    virtual void drawAdditionalGuiContent() {}

public:
    //--- Members
    const std::string description;
    const std::string linkName;
    tools::Transformation localTrafo;  //Local transformation to gripper fingers
    double fingerPercentage = 0.0;     //Between 0 and 1, where 0 is fully closed, and 1 is full open
    std::vector<robot::Visual> visuals;
    bool showGripLocation = true;
};

}  // namespace lenny::agent