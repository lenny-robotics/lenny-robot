#pragma once

#include <lenny/agents/BDSpotBaseAgent.h>

namespace lenny::agents {

class BDSpotArmRobot : public robot::Robot {
public:
    BDSpotArmRobot(const tools::Model::F_loadModel& f_loadModel = nullptr, const bool& isInStand = false)
        : robot::Robot(folderPath + "/robot.urdf", f_loadModel), isInStand(isInStand) {
        if (isInStand)
            base->loadLimitsFromFile(std::string(BDSpotBaseRobot::folderPath + "/base_limits_stand.json").c_str());
        else
            base->loadLimitsFromFile(std::string(BDSpotBaseRobot::folderPath + "/base_limits_walk.json").c_str());
    }
    inline static const std::string folderPath = LENNY_ROBOT_FOLDER "/data/bd_spot/arm";
    const bool isInStand;
};

//-----------------------------------------------------------------------------------------------------------------------------------------

/**
 * COMMENT: We optimize for the floating arm robot, and generate the leg motions of the spot base robot kinematically
 */
class BDSpotArmAgent : public BDSpotAgent {
public:
    class Gripper : public rapt::Gripper {
    public:
        enum SIDE { LEFT, RIGHT };
        Gripper(const BDSpotArmRobot& robot);
        ~Gripper() = default;

        void drawScene(const tools::Transformation& globalLinkPose, const double& alpha) const override;
    };

    BDSpotArmAgent(const std::string& name, const BDSpotArmRobot& armRobot, const BDSpotBaseRobot& baseRobot, const Eigen::VectorXd& initialArmRobotState);
    BDSpotArmAgent(const std::string& name, const BDSpotArmRobot& armRobot, const BDSpotBaseRobot& baseRobot);

    Eigen::Vector6d getSpotBasePoseFromRobotState(const Eigen::VectorXd& robotState) const override;
};

}  // namespace lenny::agents