#pragma once

#include <lenny/rapt/Gripper.h>

namespace lenny::agents {

class SakeEZGripper : public rapt::Gripper {
public:
    SakeEZGripper(const robot::Robot& robot, const std::string& linkName, const tools::Transformation& localTrafo);
    ~SakeEZGripper() = default;

    void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const override;

public:
    enum VISUALS { PALM, LINK1_LEFT, LINK2_LEFT, LINK1_RIGHT, LINK2_RIGHT };
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/sake_ezgripper";
};

}  // namespace lenny::agents