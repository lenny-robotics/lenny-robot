#pragma once

#include <lenny/robot/Robot.h>
#include <lenny/tools/Timer.h>
#include <lenny/tools/Typedefs.h>

namespace lenny::rapt {

class Gripper {
public:
    //--- Typedefs
    LENNY_GENERAGE_TYPEDEFS(Gripper)

    //--- Constructor
    Gripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo, const std::string& description = "Gripper");
    virtual ~Gripper() = default;

    //--- Helpers
    void close();
    void open();
    void setFingerPosition(const double& fingerPercentage);
    double getFingerPosition() const;

    //--- Drawing
    virtual void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const;
    void drawGui();

protected:
    virtual void drawAdditionalGuiContent() {}

private:
    void updateCurrentFingerPercentage() const;

public:
    //--- Members
    const std::string description;
    const std::string linkName;
    tools::Transformation localTrafo;  //Local transformation to gripper fingers
    std::vector<robot::Visual> visuals;
    bool showGripLocation = false;
    double fingerVelocity = 1.5;  //percentage per second

private:
    mutable tools::Timer timer;
    mutable double currentFingerPercentage = 0.5;  //Between 0 and 1, where 0 is fully closed, and 1 is full open
    double targetFingerPercentage = currentFingerPercentage;
};

}  // namespace lenny::rapt